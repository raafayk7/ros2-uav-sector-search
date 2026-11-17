"""One leader sweeping the arena against a running sim (session 8, D39).

Start the sim first with the camera bridged (the sweep reads camera_info), e.g.
  ros2 launch uav_planning swarm.launch.py num_followers:=1 followers:=false camera:=true
then
  ros2 launch uav_planning sweep.launch.py

Three processes:
  localization_node  /<robot>/pose from navsat + odometry, shared datum
  arm_takeoff        WP_SPD to wp_speed, GUIDED, arm, take off to the layer,
                     exit. The only process on Fast DDS (D15)
  sweep_node         plans the lanes from camera_info and flies them once,
                     then reports COMPLETE on /<robot>/sweep_status

The detector and leader_node are separate (uav_perception detector.launch.py,
uav_mapping heatmap.launch.py stub:=false), so the sweep runs with or without
them; it never reads detections.

Launch args:
  robot, sysid       namespace and ArduPilot sysid (default iris_1, 1)
  layer              leader altitude layer, m above home (default 8.0, D26)
  wp_speed           sweep speed, m/s, set as WP_SPD (default 5.0, D39)
  navsat_topic, odom_topic   localization inputs (default /iris_1/*, the
                     swarm launch's topics)
  camera_info_topic  '' -> /<robot>/camera/camera_info
  grid_width, grid_height, resolution, origin_x, origin_y
                     arena; must match leader_node (default 8x8 at 5 m, corner)
  overlap, lane_order, tolerance
  lane_axis          north (lanes run N-S, default) or east. Not y/x: the
                     ROS parameter parser reads a bare y as true
  datum_lat/lon/alt  shared world datum (as follower.launch.py)
  arm_timeout        wall seconds arm_takeoff keeps trying (default 600)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    robot = lc('robot')
    sysid = ParameterValue(lc('sysid'), value_type=int)
    layer = ParameterValue(lc('layer'), value_type=float)
    # Launch args are strings; ParameterValue makes them reach nodes typed.
    datum = {name: ParameterValue(lc(name), value_type=float)
             for name in ('datum_lat', 'datum_lon', 'datum_alt')}
    floats = {name: ParameterValue(lc(name), value_type=float)
              for name in ('resolution', 'origin_x', 'origin_y', 'overlap',
                           'tolerance')}
    ints = {name: ParameterValue(lc(name), value_type=int)
            for name in ('grid_width', 'grid_height')}

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='iris_1'),
        DeclareLaunchArgument('sysid', default_value='1'),
        DeclareLaunchArgument('layer', default_value='8.0'),
        DeclareLaunchArgument('wp_speed', default_value='5.0'),
        DeclareLaunchArgument('navsat_topic', default_value='/iris_1/navsat'),
        DeclareLaunchArgument('odom_topic', default_value='/iris_1/odometry'),
        DeclareLaunchArgument('camera_info_topic', default_value=''),
        DeclareLaunchArgument('grid_width', default_value='8'),
        DeclareLaunchArgument('grid_height', default_value='8'),
        DeclareLaunchArgument('resolution', default_value='5.0'),
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='0.0'),
        DeclareLaunchArgument('overlap', default_value='0.8'),
        DeclareLaunchArgument('lane_axis', default_value='north'),
        DeclareLaunchArgument('lane_order', default_value=''),
        DeclareLaunchArgument('tolerance', default_value='1.0'),
        DeclareLaunchArgument('datum_lat', default_value='-35.3632621'),
        DeclareLaunchArgument('datum_lon', default_value='149.1652374'),
        DeclareLaunchArgument('datum_alt', default_value='10.195'),
        DeclareLaunchArgument('arm_timeout', default_value='600.0'),

        Node(
            package='uav_localization', executable='localization_node',
            name='localization_node', namespace=robot, output='screen',
            parameters=[{
                'navsat_topic': lc('navsat_topic'),
                'odom_topic': lc('odom_topic'),
                **datum,
            }],
        ),
        Node(
            package='uav_planning', executable='arm_takeoff',
            name='arm_takeoff', namespace=robot, output='screen',
            # Fast DDS for this process only (D15, docker-notes gotcha 22).
            additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
            parameters=[{
                'sysid': sysid, 'alt': layer,
                'wp_speed': ParameterValue(lc('wp_speed'), value_type=float),
                'timeout': ParameterValue(lc('arm_timeout'), value_type=float),
            }],
        ),
        Node(
            package='uav_planning', executable='sweep_node',
            name='sweep_node', namespace=robot, output='screen',
            parameters=[{
                'robot': robot,
                'sysid': sysid,
                'layer': layer,
                'camera_info_topic': ParameterValue(
                    lc('camera_info_topic'), value_type=str),
                'lane_axis': lc('lane_axis'),
                # value_type=str keeps an empty or "2,0,1" order a string.
                'lane_order': ParameterValue(lc('lane_order'), value_type=str),
                **ints, **floats, **datum,
            }],
        ),
    ])
