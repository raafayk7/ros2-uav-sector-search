"""N iris in one headless Gazebo, then N followers once /clock flows (T5),
optionally led by a sweeping leader with a belief map (session 9, D40).

  ros2 launch uav_planning swarm.launch.py              # sim + 3 followers
  ros2 launch uav_planning swarm.launch.py sim:=false   # followers only
  ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1
  ros2 launch uav_planning swarm.launch.py leader:=true mission:=false

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

With a leader, mission:=true (the default) also starts mission_node (D41) and
launches every planner disabled (start_enabled false): arm_takeoff still
climbs each follower to its layer, and the mission enables the planners once
it has seen the sweep status, the map and every planner_status, then
disables them at DONE or TIMED_OUT (each drone holds, D29). mission:=false
is the session-10 behaviour: planners enabled from the start, no mission.

Launch args:
  num_followers   default 3 (the leader is extra)
  leader          add the leader, detectors and belief map (default false)
  follower_y      followers' spawn y with a leader, m (default -5.0)
  detector_seed   ground-truth detector seed, drone i gets seed + i
                  (default -1: random)
  clue            with a leader, the operator's clue (default empty = none):
                  mission_prior_node turns it into a starting belief and
                  belief_map_node waits for it (D44-D48, session 17)
  prior_provider  replay (default, no key; committed recordings in worlds/) or
                  live (llm_model picks the vendor and its key, D51)
  prior_alpha     0.5 (cap 0.7)
  prior_timeout   wall s the map waits for the prior before flying uniform
  worlds_dir      where worlds/runway/ lives (container path by default)
  llm_model       gemini-* (GEMINI_API_KEY) or claude-* (ANTHROPIC_API_KEY),
                  from the environment: docker/llm.env.example says how
  record_prior    live only: save the answer as a recording in this file
  mission         with a leader, run mission_node and start the planners
                  disabled (default true; ignored without a leader)
  mission_threshold, mission_hold   D14 end rule: every unconfirmed cell
                  below the threshold for hold wall-s (default 0.5, 30.0)
  mission_timeout sim seconds after the planners are enabled (default
                  600.0, D41)
  return_to_finds after DONE, each find's finder flies to hover over it
                  (default true; D41 amended; skipped on TIMED_OUT)
  sim             start Gazebo + SITL here (default true)
  followers       start the followers once /clock flows (default true;
                  false brings up the sim alone, e.g. to measure RTF)
  camera          bridge each drone's camera (default false)
  gui             open the Gazebo window (default false: server only; needs
                  DISPLAY and an X login file in the container)
  arena_view      draw the 40 x 40 m arena as a dark green plate with a
                  white line every 5 m on plain ground, without the runway
                  visuals (the ground collision is the same), turn shadows
                  off, colour the leader's propellers yellow and the
                  followers' cyan with a matching 1.6 m halo above each, put
                  a marker disc on the ground under each drone (starts
                  ground_marker_node and a set_pose bridge), and open the
                  Gazebo view straight down on it with a stats overlay
                  (default false; for video; use with gui:=true)
  spacing         spawn spacing along x, m (default 5.0)
  physics_step    Gazebo max_step_size, s: 0.002 (default, with a 250 Hz
                  autopilot loop) or 0.001 (upstream, 400 Hz loop)
  datum_lat/lon/alt   shared datum: navsat at the world origin, where
                  iris_1 spawns (see follower.launch.py)
  clock_timeout   wall seconds to wait for /clock (default 180)
  arm_timeout     wall seconds per drone for arm_takeoff (default 600: at
                  RTF ~0.3 the EKF takes minutes of wall time to settle)
"""

import re
import shutil
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
from launch_ros.parameter_descriptions import ParameterValue

WORLD = 'runway'
# physics step (s) -> extra SITL defaults file. Only these two are flown.
FOLLOWER_LAYER = 10.0    # D5: follower k flies at 10 + 2 k m
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


# Propeller colours with arena_view: not the target colours (red, green, blue).
PROP_COLORS = {'leader': '1 0.8 0', 'follower': '0 0.9 0.9'}
MARKER_PREFIX = 'ground_marker_'
HALO = ('<visual name="role_halo"><pose>0 0 0.06 0 0 0</pose>'
        '<cast_shadows>false</cast_shadows><transparency>0.5</transparency>'
        '<geometry><cylinder><radius>0.8</radius><length>0.01</length>'
        '</cylinder></geometry>'
        '<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse>'
        '</material></visual>')


def _iris_sdf(robot_name: str, prop_color: str = None) -> str:
    """The iris_with_gimbal SDF renamed for this drone, as iris.launch.py
    does. robot.launch.py then sets its FDM port from the instance.

    With prop_color ('r g b') the four propellers get that colour and a
    translucent disc of it floats above the drone (1.6 m across, about three
    times the airframe), through a copy of the iris_with_standoffs model.
    Visuals only, so no physics change. Upstream colours two propellers blue
    and two green."""
    models = Path(get_package_share_directory('ardupilot_gazebo'), 'models')
    sdf = (models / 'iris_with_gimbal' / 'model.sdf').read_text()
    sdf = sdf.replace('<name>iris</name>', f'<name>{robot_name}</name>')
    if prop_color:
        body = (models / 'iris_with_standoffs' / 'model.sdf').read_text()
        body, n = re.subn(r'<(ambient|diffuse)>(0 0 1|0 1 0)( 1)?</\1>',
                          rf'<\1>{prop_color} 1</\1>', body)
        uri = 'package://ardupilot_gazebo/models/iris_with_standoffs'
        base = "<visual name='base_visual'>"
        if n != 8 or sdf.count(uri) != 1 or body.count(base) != 1:
            raise RuntimeError(f'expected 8 propeller colours, one {uri} and '
                               f'one {base}, got {n}, {sdf.count(uri)} and '
                               f'{body.count(base)}')
        # A translucent disc above the propellers, so the drone shows from
        # high up. A visual on base_link has no mass or collision.
        body = body.replace(base, HALO.format(c=prop_color) + base)
        copy = Path(tempfile.mkdtemp()) / 'iris_with_standoffs'
        copy.mkdir()
        (copy / 'model.sdf').write_text(body)
        shutil.copy(models / 'iris_with_standoffs' / 'model.config', copy)
        sdf = sdf.replace(uri, f'file://{copy}')
    out = tempfile.NamedTemporaryFile('w', delete=False, suffix='.sdf')
    out.write(sdf)
    out.close()
    return out.name


# Same collision plane as upstream's runway model, a plain visual on top.
_PLAIN_GROUND_SDF = (
    '<model name="ground"><static>true</static><link name="link">'
    '<collision name="collision"><geometry><plane><normal>0 0 1</normal>'
    '</plane></geometry></collision>'
    '<visual name="ground"><cast_shadows>false</cast_shadows><geometry><plane>'
    '<normal>0 0 1</normal><size>400 400</size></plane></geometry>'
    '<material><ambient>0.6 0.6 0.55 1</ambient><diffuse>0.6 0.6 0.55 1'
    '</diffuse></material></visual></link></model>')


def _ground_marker_sdf(name: str, rgb: str, x: float, y: float) -> str:
    """A static, visual-only marker on the ground: a translucent disc with a
    solid dot in the middle. ground_marker_node moves it under its drone."""
    def disc(vname, radius, z, transparency):
        return (f'<visual name="{vname}"><pose>0 0 {z} 0 0 0</pose>'
                f'<cast_shadows>false</cast_shadows>'
                f'<transparency>{transparency}</transparency><geometry>'
                f'<cylinder><radius>{radius}</radius><length>0.01</length>'
                f'</cylinder></geometry><material><ambient>{rgb} 1</ambient>'
                f'<diffuse>{rgb} 1</diffuse></material></visual>')
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x} {y} 0.06 0 0 0</pose><link name="link">'
            + disc('ring', 0.9, 0, 0.5) + disc('dot', 0.25, 0.005, 0)
            + '</link></model>')


def _arena_gui_sdf(size: float = 40.0) -> str:
    """A <gui> block: the 3D view looking straight down at the arena centre
    (origin bottom-left, +x right, +y up), plus the stats overlay (sim time, real time, RTF). It replaces Gazebo's
    default panels, which suits a recording. Height fits the arena at the
    scene camera's default 60 degree horizontal field of view with a margin.
    """
    half = size / 2
    height = round(size * 1.2, 1)
    return (
        '<gui fullscreen="0">'
        '<plugin filename="GzScene3D" name="3D View"><gz-gui>'
        '<title>3D View</title>'
        '<property type="bool" key="showTitleBar">false</property>'
        '<property type="string" key="state">docked</property></gz-gui>'
        '<engine>ogre2</engine><scene>scene</scene>'
        '<ambient_light>0.4 0.4 0.4</ambient_light>'
        '<background_color>0.8 0.8 0.8</background_color>'
        # Yaw 90 degrees: +x to the right, +y up, origin bottom-left.
        f'<camera_pose>{half} {half} {height} 0 1.5707 1.5708</camera_pose>'
        '</plugin>'
        '<plugin filename="WorldStats" name="World stats"><gz-gui>'
        '<title>World stats</title>'
        '<property type="bool" key="showTitleBar">false</property>'
        '<property type="bool" key="resizable">false</property>'
        '<property type="double" key="height">110</property>'
        '<property type="double" key="width">290</property>'
        '<property type="double" key="z">1</property>'
        '<property type="string" key="state">floating</property>'
        '<anchors target="3D View"><line own="right" target="right"/>'
        '<line own="bottom" target="bottom"/></anchors></gz-gui>'
        '<sim_time>true</sim_time><real_time>true</real_time>'
        '<paused>false</paused><iterations>false</iterations></plugin>'
        '</gui>')


def _arena_view_sdf(size: float = 40.0, cell: float = 5.0) -> str:
    """A visual-only model: green plate over the arena with a line every cell.

    No collision and static, so physics and the planners are untouched. The
    arena is the region [0, size] x [0, size] of the world frame (D39).
    """
    half = size / 2
    n = round(size / cell)
    line = 0.15

    def box(name, x, y, sx, sy, z, rgb):
        return (f'<visual name="{name}"><pose>{x} {y} {z} 0 0 0</pose>'
                f'<cast_shadows>false</cast_shadows>'
                f'<geometry><box><size>{sx} {sy} 0.01</size></box></geometry>'
                f'<material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse>'
                f'</material></visual>')

    visuals = [box('plate', half, half, size, size, 0.03, '0.08 0.27 0.07 1')]
    for i in range(n + 1):
        at = i * cell
        visuals.append(box(f'lx{i}', at, half, line, size, 0.04, '1 1 1 1'))
        visuals.append(box(f'ly{i}', half, at, size, line, 0.04, '1 1 1 1'))
    return ('<model name="arena_view"><static>true</static>'
            '<link name="link">' + ''.join(visuals) + '</link></model>')


def _world(step: str, arena_view: bool = False, markers=()) -> Path:
    """The runway world, copied with a new max_step_size unless it is 1 ms.

    With arena_view the copy also gets the arena_view visual model and one
    ground marker per (name, rgb, x, y) in markers.
    """
    world = Path(get_package_share_directory('ardupilot_gz_gazebo'),
                 'worlds', f'{WORLD}.sdf')
    if step == '0.001' and not arena_view:
        return world
    sdf = world.read_text()
    if step != '0.001':
        old = '<max_step_size>0.001</max_step_size>'
        if sdf.count(old) != 1:
            raise RuntimeError(f'expected one {old} in {world}')
        sdf = sdf.replace(old, f'<max_step_size>{step}</max_step_size>')
    if arena_view:
        # The runway model is the world's only ground (a collision plane), so
        # swap it for a plain ground with the same collision and drop its
        # runway and airfield visuals.
        sdf, n = re.subn(r'<include>\s*<uri>model://runway</uri>.*?</include>',
                         _PLAIN_GROUND_SDF, sdf, flags=re.S)
        if n != 1 or sdf.count('</world>') != 1:
            raise RuntimeError(f'expected one runway include and </world> '
                               f'in {world}')
        # The sun is the only light: no shadows for the drones either.
        shadows = '<cast_shadows>true</cast_shadows>'
        if sdf.count(shadows) != 1 or '<gui' in sdf:
            raise RuntimeError(f'expected one {shadows} and no <gui> block '
                               f'in {world}')
        sdf = sdf.replace(shadows, '<cast_shadows>false</cast_shadows>')
        extra = ''.join(_ground_marker_sdf(*m) for m in markers)
        sdf = sdf.replace('</world>', _arena_view_sdf() + extra
                          + _arena_gui_sdf() + '</world>')
    # Same file name in a temp dir: the world name inside stays "runway".
    out = Path(tempfile.mkdtemp()) / world.name
    out.write_text(sdf)
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
    arena_view = _arg(context, 'arena_view').lower() == 'true'
    markers = [(f'{MARKER_PREFIX}{d.name}', PROP_COLORS[d.role], d.x, d.y)
               for d in drones]
    world = _world(step, arena_view, markers)
    camera = _arg(context, 'camera').lower() == 'true'
    gui = _arg(context, 'gui').lower() == 'true'
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
            # With gui:=true, no -s: the same command also opens the client.
            launch_arguments={
                'gz_args': f'-v2 {"" if gui else "-s "}-r {world}'}.items(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge', output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ]

    if arena_view:
        actions += [
            # Gazebo's set_pose service as a ROS service.
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='set_pose_bridge', output='screen',
                 arguments=[f'/world/{WORLD}/set_pose'
                            '@ros_gz_interfaces/srv/SetEntityPose']),
            Node(package='uav_planning', executable='ground_marker_node',
                 name='ground_marker_node', output='screen',
                 parameters=[{'robots': [d.name for d in drones],
                              'world': WORLD,
                              'marker_prefix': MARKER_PREFIX}]),
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
            'sdf_file': _iris_sdf(
                name, PROP_COLORS[drone.role] if arena_view else None),
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


def _mission(context, drones):
    """True when mission_node runs: a leader and mission:=true."""
    return (drones[0].role == 'leader'
            and _arg(context, 'mission').lower() == 'true')


def _follower_actions(context, drones):
    follower_file = str(Path(get_package_share_directory('uav_planning'),
                             'launch', 'follower.launch.py'))
    followers = [d for d in drones if d.role == 'follower']
    stub = len(followers) == len(drones)     # no leader: T5's stub map
    # Under a mission the planners wait for its planner_enable (D41).
    enabled = 'false' if _mission(context, drones) else 'true'
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
                'start_enabled': enabled,
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
    clue = _arg(context, 'clue').strip()
    actions.append(Node(
        package='uav_mapping', executable='belief_map_node',
        name='belief_map_node', output='screen',
        parameters=[{
            'robots': ','.join(d.name for d in drones),
            'confirmers': ','.join(d.name for d in drones if d.role == 'follower'),
            # With a clue the map waits for the prior; without one it is built
            # at once, exactly as before.
            'wait_for_prior': bool(clue),
            'prior_timeout': float(_arg(context, 'prior_timeout')),
            'use_sim_time': False,
        }]))
    if clue:
        # ParameterValue(str): a bare launch value is read as YAML otherwise
        # (docker-notes 51), and a clue is free text.
        text = lambda name: ParameterValue(_arg(context, name), value_type=str)  # noqa: E731
        actions.append(Node(
            package='uav_mapping', executable='mission_prior_node',
            name='mission_prior_node', output='screen',
            parameters=[{
                'clue': ParameterValue(clue, value_type=str),
                'world': WORLD,
                'worlds_dir': text('worlds_dir'),
                'provider': text('prior_provider'),
                'alpha': float(_arg(context, 'prior_alpha')),
                'llm_model': text('llm_model'),
                'record_to': text('record_prior'),
                'use_sim_time': False,
            }]))
    if _mission(context, drones):
        actions.append(Node(
            package='uav_planning', executable='mission_node',
            name='mission_node', output='screen',
            parameters=[{
                'followers': ','.join(d.name for d in drones if d.role == 'follower'),
                'leader': leader.name,
                'threshold': float(_arg(context, 'mission_threshold')),
                'hold': float(_arg(context, 'mission_hold')),
                'timeout': float(_arg(context, 'mission_timeout')),
                'return_to_finds': _arg(context, 'return_to_finds').lower() == 'true',
                'sysids': ','.join(f'{d.name}:{d.sysid}' for d in drones
                                   if d.role == 'follower'),
                'layers': ','.join(f'{d.name}:{FOLLOWER_LAYER + 2 * d.index}'
                                   for d in drones if d.role == 'follower'),
                'datum_lat': float(_arg(context, 'datum_lat')),
                'datum_lon': float(_arg(context, 'datum_lon')),
                'datum_alt': float(_arg(context, 'datum_alt')),
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
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('clue', default_value=''),
        DeclareLaunchArgument('prior_provider', default_value='replay'),
        DeclareLaunchArgument('prior_alpha', default_value='0.5'),
        DeclareLaunchArgument('prior_timeout', default_value='60.0'),
        DeclareLaunchArgument('worlds_dir',
                              default_value='/workspaces/multi-agent-uavs/worlds'),
        DeclareLaunchArgument('llm_model', default_value=''),
        DeclareLaunchArgument('record_prior', default_value=''),
        DeclareLaunchArgument('mission_threshold', default_value='0.5'),
        DeclareLaunchArgument('mission_hold', default_value='30.0'),
        DeclareLaunchArgument('mission_timeout', default_value='600.0'),
        DeclareLaunchArgument('return_to_finds', default_value='true'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('followers', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('arena_view', default_value='false'),
        DeclareLaunchArgument('spacing', default_value='5.0'),
        DeclareLaunchArgument('physics_step', default_value='0.002'),
        DeclareLaunchArgument('datum_lat', default_value='-35.3632621'),
        DeclareLaunchArgument('datum_lon', default_value='149.1652374'),
        DeclareLaunchArgument('datum_alt', default_value='10.195'),
        DeclareLaunchArgument('clock_timeout', default_value='180.0'),
        DeclareLaunchArgument('arm_timeout', default_value='600.0'),
        OpaqueFunction(function=_launch),
    ])
