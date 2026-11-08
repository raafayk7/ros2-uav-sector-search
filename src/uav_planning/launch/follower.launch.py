"""One follower against a running sim: localization, arming, stub map, planner.

The Nov 10 milestone launch (T3, T4). Start the sim first, e.g. headless
  ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false
then
  ros2 launch uav_planning follower.launch.py

Four processes:
  localization_node      /<robot>/pose from navsat + odometry, shared datum
  stub_heatmap_publisher /heatmap, latched (use_stub:=false when a real map
                         publisher runs)
  arm_takeoff            GUIDED, arm, take off to the layer, exit. The only
                         process on Fast DDS: /ap services hang under the
                         container's CycloneDDS (D15)
  follower_planner_node  flies to the best open sector (namespace /<robot>)

Launch args:
  robot, sysid       namespace and ArduPilot sysid (default iris_1, 1)
  peers              "robot:sysid,..." (default '' = alone)
  layer              altitude layer, m above home (default 10.0)
  navsat_topic, odom_topic   inputs for localization (iris_runway's bridge
                     publishes /navsat and /odometry; the swarm uses /iris_N/*)
  datum_lat/lon/alt  shared world datum, the runway spawn point measured from
                     /navsat on 2026-11-08 (-35.3632621, 149.1652374, 10.195)
  use_stub, start_enabled, use_sim_time
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    sysid = ParameterValue(LaunchConfiguration('sysid'), value_type=int)
    layer = ParameterValue(LaunchConfiguration('layer'), value_type=float)
    # Launch args are strings; ParameterValue makes them reach nodes as doubles.
    datum = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
             for name in ('datum_lat', 'datum_lon', 'datum_alt')}

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='iris_1'),
        DeclareLaunchArgument('sysid', default_value='1'),
        DeclareLaunchArgument('peers', default_value=''),
        DeclareLaunchArgument('layer', default_value='10.0'),
        DeclareLaunchArgument('navsat_topic', default_value='/navsat'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry'),
        DeclareLaunchArgument('datum_lat', default_value='-35.3632621'),
        DeclareLaunchArgument('datum_lon', default_value='149.1652374'),
        DeclareLaunchArgument('datum_alt', default_value='10.195'),
        DeclareLaunchArgument('use_stub', default_value='true'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='uav_localization', executable='localization_node',
            name='localization_node', namespace=robot, output='screen',
            parameters=[{
                'navsat_topic': LaunchConfiguration('navsat_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'use_sim_time': use_sim_time,
                **datum,
            }],
        ),
        Node(
            package='uav_planning', executable='stub_heatmap_publisher',
            name='stub_heatmap_publisher', output='screen',
            condition=IfCondition(LaunchConfiguration('use_stub')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='uav_planning', executable='arm_takeoff',
            name='arm_takeoff', namespace=robot, output='screen',
            # Fast DDS for this process only (D15, docker-notes gotcha 22).
            additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
            parameters=[{'sysid': sysid, 'alt': layer}],
        ),
        Node(
            package='uav_planning', executable='follower_planner_node',
            name='follower_planner_node', namespace=robot, output='screen',
            parameters=[{
                'robot': robot,
                'sysid': sysid,
                # value_type=str keeps an empty peers list a string.
                'peers': ParameterValue(LaunchConfiguration('peers'), value_type=str),
                'layer': layer,
                'start_enabled': ParameterValue(
                    LaunchConfiguration('start_enabled'), value_type=bool),
                'use_sim_time': use_sim_time,
                **datum,
            }],
        ),
    ])
