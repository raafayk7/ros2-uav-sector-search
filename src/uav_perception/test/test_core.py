"""Tests for the ROS-free detector core: projection, noise and scene parsing."""

import math

import numpy as np
import pytest

from uav_perception.core.noise import NoiseModel, frame_detections
from uav_perception.core.projection import (
    NADIR_QUAT, camera_pose, in_frame, project_ground_point, quat_to_rot)
from uav_perception.core.scene import parse_scene_models, select_targets

K = (467.8, 467.8, 320.0, 240.0)
SIZE = (640, 480)
EXACT = NoiseModel(p_miss=0.0, p_false=0.0, pixel_sigma=0.0)

# Drone 10 m up at the arena origin, nose along +x, camera straight down.
CAM_POS, WORLD_R_CAM = camera_pose((0.0, 0.0, 10.0), (0, 0, 0, 1), NADIR_QUAT)


def rng(seed=0):
    return np.random.default_rng(seed)


# ---- Projection -----------------------------------------------------------

def test_nadir_quat_matches_leader_nodes_matrix():
    # leader_node._NADIR_BODY_R_CAM, copied: the two nodes must agree.
    leader = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    assert np.allclose(quat_to_rot(*NADIR_QUAT), leader)


def test_point_below_the_drone_is_the_image_centre():
    u, v, depth = project_ground_point(0, 0, CAM_POS, WORLD_R_CAM, *K)
    assert (u, v, depth) == pytest.approx((320.0, 240.0, 10.0))


def test_forward_is_image_up_and_left_is_image_left():
    # Nadir mount: image up = body forward (+x), image right = body right (-y).
    _, v_fwd, _ = project_ground_point(2, 0, CAM_POS, WORLD_R_CAM, *K)
    u_left, _, _ = project_ground_point(0, 2, CAM_POS, WORLD_R_CAM, *K)
    assert v_fwd == pytest.approx(240.0 - 2 * 467.8 / 10)
    assert u_left == pytest.approx(320.0 - 2 * 467.8 / 10)


def test_yaw_rotates_the_picture():
    # Nose along +y (yaw 90 deg): forward is now +y, so (0, 2) is image-up.
    s = math.sqrt(0.5)
    pos, R = camera_pose((0, 0, 10), (0, 0, s, s), NADIR_QUAT)
    _, v, _ = project_ground_point(0, 2, pos, R, *K)
    assert v == pytest.approx(240.0 - 2 * 467.8 / 10)


def test_point_behind_the_camera_has_no_pixel():
    pos, R = camera_pose((0, 0, -1.0), (0, 0, 0, 1), NADIR_QUAT)
    assert project_ground_point(0, 0, pos, R, *K) is None


def test_in_frame_edges():
    assert in_frame(0, 0, 640, 480)
    assert not in_frame(640, 100, 640, 480)
    assert not in_frame(-0.1, 100, 640, 480)


# ---- One frame -------------------------------------------------------------

def test_empty_frame_is_an_empty_list():
    assert frame_detections([], CAM_POS, WORLD_R_CAM, K, SIZE, EXACT, rng()) == []


def test_visible_target_is_detected_at_its_pixel():
    (d,) = frame_detections([(2, 0)], CAM_POS, WORLD_R_CAM, K, SIZE, EXACT, rng())
    assert d.is_true
    assert (d.u, d.v) == pytest.approx((320.0, 240.0 - 2 * 467.8 / 10))
    assert 0.60 <= d.score <= 0.95


def test_target_outside_the_footprint_is_not_reported():
    # Footprint at 10 m is about 13.7 x 10.3 m: 6 m forward is beyond the top.
    assert frame_detections([(6, 0), (0, 8)], CAM_POS, WORLD_R_CAM, K, SIZE,
                            EXACT, rng()) == []


def test_box_size_follows_distance():
    near = frame_detections([(0, 0)], CAM_POS, WORLD_R_CAM, K, SIZE, EXACT, rng())
    pos, R = camera_pose((0, 0, 20.0), (0, 0, 0, 1), NADIR_QUAT)
    far = frame_detections([(0, 0)], pos, R, K, SIZE, EXACT, rng())
    assert near[0].size_px == pytest.approx(2 * far[0].size_px)


def test_certain_miss_and_certain_false_alarm():
    miss = NoiseModel(p_miss=1.0, p_false=0.0)
    assert frame_detections([(0, 0)], CAM_POS, WORLD_R_CAM, K, SIZE, miss, rng()) == []
    alarm = NoiseModel(p_miss=0.0, p_false=1.0)
    out = frame_detections([], CAM_POS, WORLD_R_CAM, K, SIZE, alarm, rng())
    assert len(out) == 1 and not out[0].is_true
    assert in_frame(out[0].u, out[0].v, *SIZE)
    assert 0.30 <= out[0].score <= 0.70


def test_miss_rate_and_false_alarm_rate_over_many_frames():
    noise = NoiseModel()                              # the agreed defaults
    g = rng(1)
    n, true, false = 20000, 0, 0
    for _ in range(n):
        for d in frame_detections([(0, 0)], CAM_POS, WORLD_R_CAM, K, SIZE, noise, g):
            true += d.is_true
            false += not d.is_true
    assert true / n == pytest.approx(1 - noise.p_miss, abs=0.02)
    assert false / n == pytest.approx(noise.p_false, abs=0.01)


def test_pixel_noise_spread():
    noise = NoiseModel(p_miss=0.0, p_false=0.0, pixel_sigma=5.0)
    g = rng(2)
    us = [frame_detections([(0, 0)], CAM_POS, WORLD_R_CAM, K, SIZE, noise, g)[0].u
          for _ in range(4000)]
    assert np.mean(us) == pytest.approx(320.0, abs=0.4)
    assert np.std(us) == pytest.approx(5.0, abs=0.3)


def test_same_seed_same_frames():
    def run(seed):
        g = rng(seed)
        return [frame_detections([(1, 1)], CAM_POS, WORLD_R_CAM, K, SIZE,
                                 NoiseModel(), g) for _ in range(50)]
    assert run(7) == run(7)
    assert run(7) != run(8)


# ---- Scene parsing -----------------------------------------------------------

# Trimmed from `gz service -s /world/runway/scene/info` (Gazebo Harmonic).
SCENE = """\
shadows: true
model {
  name: "runway"
  id: 4
  pose {
    position {
      x: -29
      y: 545
    }
    orientation {
      z: 0.02617694830787318
      w: 0.99965732497555726
    }
  }
  link {
    id: 5
    name: "link"
    pose {
      position {
        x: 99
      }
    }
  }
}
model {
  name: "iris_1"
  id: 11
  pose {
    position {
      x: -1.6420971496896959e-22
      z: 0.19496080000000002
    }
  }
  model {
    name: "nested"
    pose {
      position {
        x: 50
      }
    }
  }
}
model {
  name: "target_a"
  id: 67
  pose {
    position {
      x: 3.5
      y: -2
      z: 0.05
    }
  }
  link {
    name: "l"
    pose {
      position {
        y: 77
      }
    }
  }
}
model {
  name: "target_origin"
  pose {
    orientation {
      w: 1
    }
  }
}
light {
  name: "sun"
}
"""


def test_parse_scene_reads_top_level_model_poses_only():
    models = parse_scene_models(SCENE)
    assert set(models) == {'runway', 'iris_1', 'target_a', 'target_origin'}
    assert models['runway'] == (-29.0, 545.0, 0.0)
    assert models['target_a'] == (3.5, -2.0, 0.05)
    assert models['iris_1'][0] == pytest.approx(0.0, abs=1e-20)


def test_omitted_fields_mean_zero():
    assert parse_scene_models(SCENE)['target_origin'] == (0.0, 0.0, 0.0)


def test_select_targets_by_prefix():
    assert select_targets(parse_scene_models(SCENE)) == [
        ('target_a', 3.5, -2.0), ('target_origin', 0.0, 0.0)]
    assert select_targets(parse_scene_models(SCENE), prefix='iris') == [
        ('iris_1', pytest.approx(0.0, abs=1e-20), pytest.approx(0.0, abs=1e-20))]


def test_empty_scene():
    assert parse_scene_models('') == {}
