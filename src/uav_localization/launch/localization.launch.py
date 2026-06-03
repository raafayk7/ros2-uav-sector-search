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
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navsat_topic = LaunchConfiguration('navsat_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    robot = LaunchConfiguration('robot')
    arena_frame = LaunchConfiguration('arena_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('navsat_topic', default_value='/navsat'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry'),
        DeclareLaunchArgument('pose_topic', default_value='pose'),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('arena_frame', default_value='arena'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

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
            }],
        ),
    ])
