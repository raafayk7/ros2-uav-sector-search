"""Bring up leader_node + the fake-detection stub for heatmap testing.

Mirrors localization.launch.py's argument conventions. Brings up the
detection-to-heatmap node and a synthetic detection publisher so you can test
the projection + accumulation without perception_node existing yet.

Typical uses
------------
Standalone (NO simulator -- stub provides its own camera_info):
  ros2 launch uav_mapping heatmap.launch.py \\
    standalone:=true det_u:=320.0 det_v:=240.0

Against iris_runway (sim provides /camera/camera_info; you provide /pose via
localization_node, and drag the iris in Gazebo):
  ros2 launch uav_mapping heatmap.launch.py det_u:=320.0 det_v:=240.0

Key args
--------
  pose_topic        leader pose          (default /pose)
  detections_topic  detection channel    (default /detections)
  camera_info_topic intrinsics           (default /camera/camera_info)
  heatmap_topic     output map           (default /heatmap, project-level)
  arena_frame       frame_id on /heatmap (default arena)
  resolution        m per cell           (default 5.0  -- thesis sector size)
  grid_width        cells in x/East      (default 4)
  grid_height       cells in y/North     (default 4)
  origin_x/origin_y grid corner in world (default 0,0 -- CORNER placement,
                                          thesis-style; -10,-10 = centered)
  standalone        stub emits camera_info too (default false)
  det_u/det_v       stub detection pixel (default 320, 240)
  det_score         stub detection score (default 0.9)
  use_sim_time      use /clock           (default true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    lc = LaunchConfiguration

    # Standalone mode runs with no simulator, so nothing publishes /clock.
    # A node with use_sim_time=true would freeze (its timers wait on a clock
    # that never advances), so force sim time OFF whenever standalone is true;
    # otherwise honour the use_sim_time arg (true for the iris_runway case).
    sim_time = PythonExpression(
        ["'false' if '", lc('standalone'), "' == 'true' else '",
         lc('use_sim_time'), "'"])

    args = [
        DeclareLaunchArgument('pose_topic', default_value='/pose'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/camera_info'),
        DeclareLaunchArgument('heatmap_topic', default_value='/heatmap'),
        DeclareLaunchArgument('arena_frame', default_value='arena'),
        DeclareLaunchArgument('resolution', default_value='5.0'),
        DeclareLaunchArgument('grid_width', default_value='4'),
        DeclareLaunchArgument('grid_height', default_value='4'),
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='0.0'),
        DeclareLaunchArgument('standalone', default_value='false'),
        DeclareLaunchArgument('det_u', default_value='320.0'),
        DeclareLaunchArgument('det_v', default_value='240.0'),
        DeclareLaunchArgument('det_score', default_value='0.9'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    leader = Node(
        package='uav_mapping',
        executable='leader_node',
        name='leader_node',
        output='screen',
        parameters=[{
            'pose_topic': lc('pose_topic'),
            'detections_topic': lc('detections_topic'),
            'camera_info_topic': lc('camera_info_topic'),
            'heatmap_topic': lc('heatmap_topic'),
            'arena_frame': lc('arena_frame'),
            'resolution': lc('resolution'),
            'grid_width': lc('grid_width'),
            'grid_height': lc('grid_height'),
            'origin_x': lc('origin_x'),
            'origin_y': lc('origin_y'),
            'use_sim_time': sim_time,
        }],
    )

    stub = Node(
        package='uav_mapping',
        executable='fake_detection_publisher',
        name='fake_detection_publisher',
        output='screen',
        parameters=[{
            'detections_topic': lc('detections_topic'),
            'camera_info_topic': lc('camera_info_topic'),
            # 'standalone' flips the stub's own camera_info publisher on, so
            # leader_node has intrinsics even with no simulator running.
            'publish_camera_info': lc('standalone'),
            'u': lc('det_u'),
            'v': lc('det_v'),
            'score': lc('det_score'),
            'use_sim_time': sim_time,
        }],
    )

    return LaunchDescription(args + [leader, stub])
