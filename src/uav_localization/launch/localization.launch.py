"""Unit-bringup launch for the localization node.

Brings up *only* localization_node, for testing it in isolation against a
running iris_runway sim before wiring it into a bigger swarm launch.

Launch args (all overridable on the command line, e.g.
  ros2 launch uav_localization localization.launch.py robot:=iris_1):

  navsat_topic  input GPS topic    (default /navsat   — ros_gz_bridge)
  odom_topic    input odom topic   (default /odometry — ros_gz_bridge)
  pose_topic    output pose topic  (default pose      — relative, so it
                                     becomes /<robot>/pose under a namespace)
  robot         node namespace     (default ''        — unnamespaced today;
                                     set to iris_1 etc. for the swarm)
  arena_frame   frame_id on /pose  (default arena)
  use_sim_time  use /clock         (default true      — Gazebo drives time)
  datum_lat/lon/alt  shared world datum (default nan = unset: latch the first
                     fix). Set all three on every drone of a swarm so they
                     share one frame, e.g. datum_lat:=-35.3632621
                     datum_lon:=149.1652374 datum_alt:=10.19 (runway world).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navsat_topic = LaunchConfiguration('navsat_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    robot = LaunchConfiguration('robot')
    arena_frame = LaunchConfiguration('arena_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Launch args are strings; ParameterValue makes them reach the node as doubles.
    datum = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
             for name in ('datum_lat', 'datum_lon', 'datum_alt')}

    return LaunchDescription([
        DeclareLaunchArgument('navsat_topic', default_value='/navsat'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry'),
        DeclareLaunchArgument('pose_topic', default_value='pose'),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('arena_frame', default_value='arena'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('datum_lat', default_value='nan'),
        DeclareLaunchArgument('datum_lon', default_value='nan'),
        DeclareLaunchArgument('datum_alt', default_value='nan'),

        Node(
            package='uav_localization',
            executable='localization_node',
            name='localization_node',
            namespace=robot,
            output='screen',
            parameters=[{
                'navsat_topic': navsat_topic,
                'odom_topic': odom_topic,
                'pose_topic': pose_topic,
                'arena_frame': arena_frame,
                # Honour Gazebo's /clock so timestamps line up with the sim.
                'use_sim_time': use_sim_time,
                **datum,
            }],
        ),
    ])
