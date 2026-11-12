"""N iris in one headless Gazebo, then N followers once /clock flows (T5).

  ros2 launch uav_planning swarm.launch.py              # sim + 3 followers
  ros2 launch uav_planning swarm.launch.py sim:=false   # followers only

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

Drone i (0-based) is iris_{i+1}, sysid i+1, SITL instance i, spawned at
x = i * spacing on the runway, altitude layer 10 + 2 i m (D5). Every drone
lists all others as peers, and all share one datum (D7). One stub map.

Launch args:
  num_followers   default 3
  sim             start Gazebo + SITL here (default true)
  camera          bridge each drone's camera (default false)
  spacing         spawn spacing along x, m (default 5.0)
  datum_lat/lon/alt   shared datum: navsat at the world origin, where
                  iris_1 spawns (see follower.launch.py)
  clock_timeout   wall seconds to wait for /clock (default 180)
  arm_timeout     wall seconds per drone for arm_takeoff (default 600: at
                  RTF ~0.3 the EKF takes minutes of wall time to settle)
"""

import tempfile
from pathlib import Path

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


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _defaults(source, context):
    """Default values of a launch file's arguments (as upstream does)."""
    ld = LaunchDescription([IncludeLaunchDescription(source)])
    return {a.name: a.default_value[0].perform(context)
            for a in ld.get_launch_arguments()}


def _bridge_config(camera: bool) -> str:
    """Upstream's iris bridge config minus clock, gz/tf and (optionally)
    camera, written to a temp file. robot.launch.py fills in the
    {{ robot_name }} and {{ world_name }} placeholders."""
    src = Path(get_package_share_directory('ardupilot_gz_bringup'),
               'config', 'iris_bridge.yaml')
    entries = yaml.safe_load(src.read_text())
    keep = [e for e in entries
            if e['ros_topic_name'] != 'clock'
            and not e['ros_topic_name'].startswith('gz/')
            and (camera or not e['ros_topic_name'].startswith('camera/'))]
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


def _sim_actions(context, n):
    bringup = get_package_share_directory('ardupilot_gz_bringup')
    world = Path(get_package_share_directory('ardupilot_gz_gazebo'),
                 'worlds', f'{WORLD}.sdf')
    spacing = float(_arg(context, 'spacing'))
    bridge_file = _bridge_config(_arg(context, 'camera').lower() == 'true')

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

    for i in range(n):
        name = f'iris_{i + 1}'
        drone_args = dict(base)
        drone_args.update({
            'namespace': name,
            'robot_name': name,
            'world_name': WORLD,
            'sdf_file': _iris_sdf(name),
            'bridge_config_file': bridge_file,
            'command': 'arducopter',
            'use_gz_tf': 'false',
            'x': str(i * spacing), 'y': '0.0', 'z': '0.195',
            'R': '0', 'P': '0', 'Y': '1.5708',
            'instance': str(i),
            'sysid': str(i + 1),
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


def _follower_actions(context, n):
    follower_file = str(Path(get_package_share_directory('uav_planning'),
                             'launch', 'follower.launch.py'))
    names = [(f'iris_{i + 1}', i + 1) for i in range(n)]
    actions = []
    for i, (name, sysid) in enumerate(names):
        peers = ','.join(f'{p}:{s}' for p, s in names if p != name)
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
                'use_stub': 'true' if i == 0 else 'false',
                'arm_timeout': _arg(context, 'arm_timeout'),
            }.items()))
    return actions


def _launch(context):
    n = int(_arg(context, 'num_followers'))
    actions = _sim_actions(context, n) if _arg(context, 'sim').lower() == 'true' else []

    gate = Node(
        package='uav_planning', executable='wait_for_clock',
        name='wait_for_clock', output='screen',
        parameters=[{'timeout': float(_arg(context, 'clock_timeout'))}],
    )

    def on_gate_exit(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='/clock never flowed; not starting followers'),
                    Shutdown(reason='no /clock')]
        return [LogInfo(msg=f'/clock flowing; starting {n} followers')] + \
            _follower_actions(context, n)

    actions += [gate, RegisterEventHandler(
        OnProcessExit(target_action=gate, on_exit=on_gate_exit))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_followers', default_value='3'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('spacing', default_value='5.0'),
        DeclareLaunchArgument('datum_lat', default_value='-35.3632621'),
        DeclareLaunchArgument('datum_lon', default_value='149.1652374'),
        DeclareLaunchArgument('datum_alt', default_value='10.195'),
        DeclareLaunchArgument('clock_timeout', default_value='180.0'),
        DeclareLaunchArgument('arm_timeout', default_value='600.0'),
        OpaqueFunction(function=_launch),
    ])
