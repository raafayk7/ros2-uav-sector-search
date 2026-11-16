"""Write a patched copy of swarm.launch.py for one T7 (D23) speed probe.

  python3 docs/probes/rtf_variant.py [nomav] [nocam]   # prints the path

nomav: no MAVProxy for any SITL. Upstream starts one per instance (in
  sitl_mavproxy.launch.py and sitl_dds_udp.launch.py). Without it SITL blocks
  on SERIAL0 (tcp:0:wait, gotcha 31), so serial0 becomes tcp:0, which listens
  without waiting.
nocam: the gimbal without its camera sensor (and Gst/zoom plugins). Probe
  only: whether followers may drop the camera is a design question.
lean: no joint_states and no imu in the per-drone ROS bridge. Gazebo publishes
  both every physics step, and a bridged topic must be serialized every step.

The 2 ms physics step this probe first tried (step2) is now swarm.launch.py's
default (D37): pass physics_step:=0.001 to rtf_swarm.sh for upstream timing.

Run inside the container. Copies go to /tmp; nothing installed is changed.
"""

import re
import shutil
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

flags = set(sys.argv[1:])
assert flags <= {'nomav', 'nocam', 'lean'}, flags
out = Path('/tmp/rtf_variant')
shutil.rmtree(out, ignore_errors=True)
out.mkdir()

swarm = Path(get_package_share_directory('uav_planning'),
             'launch', 'swarm.launch.py').read_text()

if 'nomav' in flags:
    sitl_launch = Path(get_package_share_directory('ardupilot_sitl'), 'launch')
    dds = (sitl_launch / 'sitl_dds_udp.launch.py').read_text()
    dds, c = re.subn(r'\n\s*mavproxy,\n', '\n', dds)
    assert c == 1
    (out / 'sitl_dds_udp_nomav.launch.py').write_text(dds)

    robot = Path(get_package_share_directory('ardupilot_gz_bringup'),
                 'launch', 'robots', 'robot.launch.py').read_text()
    for old, new in [('"sitl_mavproxy.launch.py"', '"sitl.launch.py"'),
                     ('"sitl_dds_udp.launch.py"',
                      f'"{out}/sitl_dds_udp_nomav.launch.py"'),
                     ('"use_instance_dir": LaunchConfiguration("use_instance_dir"),',
                      '"use_instance_dir": LaunchConfiguration("use_instance_dir"),\n'
                      '      "serial0": "tcp:0",')]:
        assert robot.count(old) == 1, old
        robot = robot.replace(old, new)
    (out / 'robot_nomav.launch.py').write_text(robot)
    old = "str(Path(bringup, 'launch', 'robots', 'robot.launch.py'))"
    assert swarm.count(old) == 1
    swarm = swarm.replace(old, f"'{out}/robot_nomav.launch.py'")

if 'nocam' in flags:
    models = Path(get_package_share_directory('ardupilot_gazebo'), 'models')
    gimbal = out / 'gimbal_small_3d_nocam'
    shutil.copytree(models / 'gimbal_small_3d', gimbal)
    sdf = (gimbal / 'model.sdf').read_text()
    sdf, c = re.subn(r'<sensor name="camera" type="camera">.*?</sensor>', '',
                     sdf, flags=re.S)
    assert c == 1
    (gimbal / 'model.sdf').write_text(sdf)
    old = "sdf = sdf.replace('<name>iris</name>', f'<name>{robot_name}</name>')"
    assert swarm.count(old) == 1
    swarm = swarm.replace(old, old + "\n    sdf = sdf.replace("
                          "'package://ardupilot_gazebo/models/gimbal_small_3d', "
                          f"'{gimbal}')")

if 'lean' in flags:
    old = "            and not e['ros_topic_name'].startswith('gz/')"
    assert swarm.count(old) == 1
    swarm = swarm.replace(old, old + "\n            and e['ros_topic_name'] not in ('joint_states', 'imu')")

path = out / 'swarm_variant.launch.py'
path.write_text(swarm)
print(path)
