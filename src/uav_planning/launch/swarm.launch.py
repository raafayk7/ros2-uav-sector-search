"""N iris in one headless Gazebo, then N followers once /clock flows (T5),
optionally led by a sweeping leader with a belief map (session 9, D40).

  ros2 launch uav_planning swarm.launch.py              # sim + 3 followers
  ros2 launch uav_planning swarm.launch.py sim:=false   # followers only
  ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1

Why our own sim launch: upstream multiagent.launch.py hard-codes 4 different
vehicles. This reuses its per-vehicle building block, robots/robot.launch.py
(spawn, SITL, bridge, robot_state_publisher), with three changes:

  - One /clock bridge in the global namespace. Upstream's iris bridge config
    bridges "clock" as a relative topic, so with namespaces it lands on
    /iris_N/clock and nothing publishes /clock: every use_sim_time node
    would freeze. Measured 2026-11-12 with 3 iris.
  - A trimmed per-drone bridge: no clock, no gz/tf, and no camera unless
    camera:=true. Each bridged topic costs CPU at sensor rate, and the
    planner needs only navsat and odometry.
  - Our nodes start only after wait_for_clock exits 0 (docker-notes gotcha
    33). If /clock never flows the launch shuts down with an error instead
    of leaving planners that never tick.
  - config/gimbal_nadir.parm in each SITL's defaults, so the camera looks
    straight down (T6, D30). It goes right after the gimbal file and before
    the dds_*.parm files; last in the list, it silently drops DDS_USE_NS
    (see the parm file).
  - A 2 ms physics step by default instead of the world's 1 ms (D37). One
    Gazebo thread spends about 1 ms of CPU per drone per step, so halving the
    steps per sim second doubles RTF (0.24 -> 0.48 at 4 idle iris, T7). The
    IMU then samples at 500 Hz, too slow for ArduCopter's 400 Hz loop, so
    config/loop_250hz.parm goes in with it. physics_step:=0.001 restores
    upstream timing for any run where fidelity is in doubt.

Without a leader (default, the T5 launch): drone i (0-based) is iris_{i+1},
sysid i+1, SITL instance i, spawned at x = i * spacing on the runway,
altitude layer 10 + 2 i m (D5). Every drone lists all others as peers, and
all share one datum (D7). One stub map.

With leader:=true (D40): iris_1, sysid 1, is the leader. It spawns at the
world origin, which is the arena corner and the datum, and runs
sweep.launch.py (8 m layer, 5 m/s, D39). Followers are iris_2.., sysid 2..,
layers 10 + 2 k m for follower k = 0, 1, ..., spawned at x = (k + 1) *
spacing on a line follower_y m from the arena edge (default 5 m south), so
no follower climbs through the leader's 8 m layer near its path. Followers
list only each other as peers: the planner counts a peer with no goal as
parked where it is, so the leader would block wherever it flew. Every drone
gets a ground-truth detector in its namespace, and belief_map_node fuses them
all into /heatmap in place of the stub. The leader bridges camera_info (the
sweep plans from it); the followers' detectors and the map use the sim-camera
defaults, so they bridge no camera unless camera:=true.

Launch args:
  num_followers   default 3 (the leader is extra)
  leader          add the leader, detectors and belief map (default false)
  follower_y      followers' spawn y with a leader, m (default -5.0)
  detector_seed   ground-truth detector seed, drone i gets seed + i
                  (default -1: random)
  sim             start Gazebo + SITL here (default true)
  followers       start the followers once /clock flows (default true;
                  false brings up the sim alone, e.g. to measure RTF)
  camera          bridge each drone's camera (default false)
  spacing         spawn spacing along x, m (default 5.0)
  physics_step    Gazebo max_step_size, s: 0.002 (default, with a 250 Hz
                  autopilot loop) or 0.001 (upstream, 400 Hz loop)
  datum_lat/lon/alt   shared datum: navsat at the world origin, where
                  iris_1 spawns (see follower.launch.py)
  clock_timeout   wall seconds to wait for /clock (default 180)
  arm_timeout     wall seconds per drone for arm_takeoff (default 600: at
                  RTF ~0.3 the EKF takes minutes of wall time to settle)
"""

import tempfile
from pathlib import Path
from typing import NamedTuple

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = 'runway'
# physics step (s) -> extra SITL defaults file. Only these two are flown.
STEP_PARMS = {'0.001': None, '0.002': 'loop_250hz.parm'}


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _defaults(source, context):
    """Default values of a launch file's arguments (as upstream does)."""
    ld = LaunchDescription([IncludeLaunchDescription(source)])
    return {a.name: a.default_value[0].perform(context)
            for a in ld.get_launch_arguments()}


class Drone(NamedTuple):
    name: str
    sysid: int
    role: str         # 'leader' or 'follower'
    index: int        # follower index k (layer 10 + 2 k); 0 for the leader
    x: float          # spawn, Gazebo world = arena frame
    y: float


def _drones(context):
    n = int(_arg(context, 'num_followers'))
    spacing = float(_arg(context, 'spacing'))
    if _arg(context, 'leader').lower() != 'true':
        return [Drone(f'iris_{i + 1}', i + 1, 'follower', i, i * spacing, 0.0)
                for i in range(n)]
    y = float(_arg(context, 'follower_y'))
    return [Drone('iris_1', 1, 'leader', 0, 0.0, 0.0)] + [
        Drone(f'iris_{k + 2}', k + 2, 'follower', k, (k + 1) * spacing, y)
        for k in range(n)]


def _bridge_config(camera: str) -> str:
    """Upstream's iris bridge config minus clock, gz/tf and camera topics,
    written to a temp file. camera: 'none', 'info' (camera_info only) or
    'all'. robot.launch.py fills in the {{ robot_name }} and
    {{ world_name }} placeholders."""
    src = Path(get_package_share_directory('ardupilot_gz_bringup'),
               'config', 'iris_bridge.yaml')
    entries = yaml.safe_load(src.read_text())
    cams = {'none': (), 'info': ('camera/camera_info',),
            'all': ('camera/camera_info', 'camera/image')}[camera]
    keep = [e for e in entries
            if e['ros_topic_name'] != 'clock'
            and not e['ros_topic_name'].startswith('gz/')
            and (not e['ros_topic_name'].startswith('camera/')
                 or e['ros_topic_name'] in cams)]
    out = tempfile.NamedTemporaryFile('w', delete=False, suffix='.yaml')
    yaml.safe_dump(keep, out, sort_keys=False)
    out.close()
    return out.name


def _iris_sdf(robot_name: str) -> str:
    """The iris_with_gimbal SDF renamed for this drone, as iris.launch.py
    does. robot.launch.py then sets its FDM port from the instance."""
    sdf = Path(get_package_share_directory('ardupilot_gazebo'),
               'models', 'iris_with_gimbal', 'model.sdf').read_text()
    sdf = sdf.replace('<name>iris</name>', f'<name>{robot_name}</name>')
    out = tempfile.NamedTemporaryFile('w', delete=False, suffix='.sdf')
    out.write(sdf)
    out.close()
    return out.name


def _world(step: str) -> Path:
    """The runway world, copied with a new max_step_size unless it is 1 ms."""
    world = Path(get_package_share_directory('ardupilot_gz_gazebo'),
                 'worlds', f'{WORLD}.sdf')
    if step == '0.001':
        return world
    sdf = world.read_text()
    old = '<max_step_size>0.001</max_step_size>'
    if sdf.count(old) != 1:
        raise RuntimeError(f'expected one {old} in {world}')
    # Same file name in a temp dir: the world name inside stays "runway".
    out = Path(tempfile.mkdtemp()) / world.name
    out.write_text(sdf.replace(old, f'<max_step_size>{step}</max_step_size>'))
    return out


def _with_ours(defaults: str, step: str) -> str:
    """Insert gimbal_nadir.parm (and the loop-rate file for this physics
    step) right after upstream's gimbal parm file, so they override it but a
    dds_*.parm file stays last."""
    config = Path(get_package_share_directory('uav_planning'), 'config')
    ours = [str(config / 'gimbal_nadir.parm')]
    if STEP_PARMS[step]:
        ours.append(str(config / STEP_PARMS[step]))
    files = defaults.split(',')
    gimbal = [i for i, f in enumerate(files)
              if f.endswith('gazebo-iris-gimbal.parm')]
    if not gimbal:
        raise RuntimeError(f'no gazebo-iris-gimbal.parm in defaults: {defaults}')
    files[gimbal[0] + 1:gimbal[0] + 1] = ours
    return ','.join(files)


def _sim_actions(context, drones):
    bringup = get_package_share_directory('ardupilot_gz_bringup')
    step = _arg(context, 'physics_step')
    if step not in STEP_PARMS:
        raise RuntimeError(f'physics_step must be one of {list(STEP_PARMS)}, '
                           f'got {step}')
    world = _world(step)
    camera = _arg(context, 'camera').lower() == 'true'
    bridge_files = {
        'follower': _bridge_config('all' if camera else 'none'),
        'leader': _bridge_config('all' if camera else 'info'),
    }

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'))),
            # -s server only (headless), -r run at start, -v2 warnings only.
            launch_arguments={'gz_args': f'-v2 -s -r {world}'}.items(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge', output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ]

    robot_lds = PythonLaunchDescriptionSource(
        str(Path(bringup, 'launch', 'robots', 'robot.launch.py')))
    iris_lds = PythonLaunchDescriptionSource(
        str(Path(bringup, 'launch', 'robots', 'iris.launch.py')))
    base = _defaults(robot_lds, context)
    # iris.launch.py's defaults: copter + gimbal + DDS param files.
    base.update({k: v for k, v in _defaults(iris_lds, context).items()
                 if k in ('model', 'defaults', 'synthetic_clock', 'sim_address')})
    base['defaults'] = _with_ours(base['defaults'], step)

    for i, drone in enumerate(drones):
        name = drone.name
        drone_args = dict(base)
        drone_args.update({
            'namespace': name,
            'robot_name': name,
            'world_name': WORLD,
            'sdf_file': _iris_sdf(name),
            'bridge_config_file': bridge_files[drone.role],
            'command': 'arducopter',
            'use_gz_tf': 'false',
            'x': str(drone.x), 'y': str(drone.y), 'z': '0.195',
            'R': '0', 'P': '0', 'Y': '1.5708',
            'instance': str(i),
            'sysid': str(drone.sysid),
            'use_instance_dir': 'True',
            # One MicroXRCEAgent serves every SITL; AP_DDS namespaces by sysid.
            'use_dds_agent': 'True' if i == 0 else 'False',
        })
        # Not in a scoped GroupAction: robot.launch.py reads use_gz_tf in a
        # deferred OnProcessStart handler, after a scope would have dropped
        # it. Every argument is passed explicitly, so nothing leaks between
        # drones that matters.
        actions.append(IncludeLaunchDescription(
            robot_lds, launch_arguments=drone_args.items()))
    return actions


def _follower_actions(context, drones):
    follower_file = str(Path(get_package_share_directory('uav_planning'),
                             'launch', 'follower.launch.py'))
    followers = [d for d in drones if d.role == 'follower']
    stub = len(followers) == len(drones)     # no leader: T5's stub map
    actions = []
    for d in followers:
        name, sysid, i = d.name, d.sysid, d.index
        peers = ','.join(f'{p.name}:{p.sysid}' for p in followers if p != d)
        # A fresh source per drone: a source caches its parsed file, and
        # follower.launch.py builds its Nodes at parse time, so a shared
        # source would hand every drone the same Node objects. Unscoped like
        # the sim includes; every argument is explicit.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follower_file), launch_arguments={
                'robot': name,
                'sysid': str(sysid),
                'peers': peers,
                'layer': str(10.0 + 2.0 * i),
                'navsat_topic': f'/{name}/navsat',
                'odom_topic': f'/{name}/odometry',
                'datum_lat': _arg(context, 'datum_lat'),
                'datum_lon': _arg(context, 'datum_lon'),
                'datum_alt': _arg(context, 'datum_alt'),
                'use_stub': 'true' if stub and i == 0 else 'false',
                'arm_timeout': _arg(context, 'arm_timeout'),
            }.items()))
    return actions


def _leader_actions(context, drones):
    """The leader's sweep, a detector per drone and the belief map (D40)."""
    share = get_package_share_directory('uav_planning')
    leader = drones[0]
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share, 'launch', 'sweep.launch.py'))),
        launch_arguments={
            'robot': leader.name,
            'sysid': str(leader.sysid),
            'navsat_topic': f'/{leader.name}/navsat',
            'odom_topic': f'/{leader.name}/odometry',
            'datum_lat': _arg(context, 'datum_lat'),
            'datum_lon': _arg(context, 'datum_lon'),
            'datum_alt': _arg(context, 'datum_alt'),
            'arm_timeout': _arg(context, 'arm_timeout'),
        }.items())]
    seed = int(_arg(context, 'detector_seed'))
    for i, d in enumerate(drones):
        actions.append(Node(
            package='uav_perception', executable='ground_truth_detector',
            name='ground_truth_detector', namespace=d.name, output='screen',
            parameters=[{
                'pose_topic': f'/{d.name}/pose',
                'detections_topic': f'/{d.name}/detections',
                'camera_info_topic': f'/{d.name}/camera/camera_info',
                'seed': seed + i if seed >= 0 else -1,
                'use_sim_time': False,
            }]))
    actions.append(Node(
        package='uav_mapping', executable='belief_map_node',
        name='belief_map_node', output='screen',
        parameters=[{
            'robots': ','.join(d.name for d in drones),
            'confirmers': ','.join(d.name for d in drones if d.role == 'follower'),
            'use_sim_time': False,
        }]))
    return actions


def _launch(context):
    drones = _drones(context)
    n = len(drones)
    actions = _sim_actions(context, drones) if _arg(context, 'sim').lower() == 'true' else []
    if _arg(context, 'followers').lower() != 'true':
        return actions

    gate = Node(
        package='uav_planning', executable='wait_for_clock',
        name='wait_for_clock', output='screen',
        parameters=[{'timeout': float(_arg(context, 'clock_timeout'))}],
    )

    def on_gate_exit(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='/clock never flowed; not starting followers'),
                    Shutdown(reason='no /clock')]
        started = _follower_actions(context, drones)
        if drones[0].role == 'leader':
            started = _leader_actions(context, drones) + started
        return [LogInfo(msg=f'/clock flowing; starting {n} drones\' nodes')] + started

    actions += [gate, RegisterEventHandler(
        OnProcessExit(target_action=gate, on_exit=on_gate_exit))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_followers', default_value='3'),
        DeclareLaunchArgument('leader', default_value='false'),
        DeclareLaunchArgument('follower_y', default_value='-5.0'),
        DeclareLaunchArgument('detector_seed', default_value='-1'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('followers', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('spacing', default_value='5.0'),
        DeclareLaunchArgument('physics_step', default_value='0.002'),
        DeclareLaunchArgument('datum_lat', default_value='-35.3632621'),
        DeclareLaunchArgument('datum_lon', default_value='149.1652374'),
        DeclareLaunchArgument('datum_alt', default_value='10.195'),
        DeclareLaunchArgument('clock_timeout', default_value='180.0'),
        DeclareLaunchArgument('arm_timeout', default_value='600.0'),
        OpaqueFunction(function=_launch),
    ])
