"""The ground-truth detector (a YOLO stand-in) on its own.

  ros2 launch uav_perception detector.launch.py
  ros2 launch uav_perception detector.launch.py target_source:=param \
      targets_xy:='[3.0, 4.0, 12.0, 7.5]'

Against a running sim the targets are the Gazebo models named target_*
(spawn them as docs/probes/mount_check.py does). Pair it with leader_node
(uav_mapping heatmap.launch.py) and localization_node for /pose. Noise values
are the session 7 defaults; seed:=N makes a run repeatable.

It uses wall time (use_sim_time false): a /clock subscriber costs about 45 %
of a core in a swarm (docker-notes 36), and a frame here is a timer tick.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as lc
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'pose_topic': '/pose',
        'detections_topic': '/detections',
        'camera_info_topic': '/camera/camera_info',
        'target_source': 'gz',
        'frame_rate': '2.0',
        'p_miss': '0.20',
        'p_false': '0.05',
        'pixel_sigma': '5.0',
        'seed': '-1',
    }
    args = [DeclareLaunchArgument(k, default_value=v)
            for k, v in defaults.items()]
    args.append(DeclareLaunchArgument('targets_xy', default_value='[0.0]'))

    node = Node(
        package='uav_perception', executable='ground_truth_detector',
        name='ground_truth_detector', output='screen',
        parameters=[{
            'pose_topic': lc('pose_topic'),
            'detections_topic': lc('detections_topic'),
            'camera_info_topic': lc('camera_info_topic'),
            'target_source': lc('target_source'),
            # Launch args are strings: say which type each parameter is.
            'frame_rate': ParameterValue(lc('frame_rate'), value_type=float),
            'p_miss': ParameterValue(lc('p_miss'), value_type=float),
            'p_false': ParameterValue(lc('p_false'), value_type=float),
            'pixel_sigma': ParameterValue(lc('pixel_sigma'), value_type=float),
            'seed': ParameterValue(lc('seed'), value_type=int),
            'targets_xy': ParameterValue(lc('targets_xy'),
                                         value_type=list[float]),
            'use_sim_time': False,
        }])
    return LaunchDescription(args + [node])
