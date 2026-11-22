"""arena_view pieces: the marker node's move rule and the world it needs.

The world tests load swarm.launch.py and read upstream's runway.sdf, so they
need ROS sourced (like the datum tests).
"""

import importlib.util
from pathlib import Path

import pytest

from uav_planning.ground_marker_node import moved_enough

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'swarm.launch.py'


@pytest.fixture(scope='module')
def swarm():
    spec = importlib.util.spec_from_file_location('swarm_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_first_request_always_goes_out():
    assert moved_enough(None, (0.0, 0.0), 0.05)


def test_a_hovering_drone_sends_nothing():
    assert not moved_enough((10.0, 5.0), (10.01, 5.02), 0.05)


def test_a_moving_drone_sends():
    assert moved_enough((10.0, 5.0), (10.0, 5.06), 0.05)


def test_marker_model_is_static_and_visual_only(swarm):
    sdf = swarm._ground_marker_sdf('ground_marker_iris_2', '0 0.9 0.9', 5, -5)
    assert '<model name="ground_marker_iris_2">' in sdf
    assert '<static>true</static>' in sdf
    assert '<pose>5 -5 0.06' in sdf
    assert '<collision' not in sdf and '<inertial' not in sdf


def test_world_with_arena_view_has_one_marker_per_drone(swarm):
    markers = [(f'ground_marker_iris_{i}', '1 0.8 0', i * 5, 0)
               for i in range(1, 5)]
    text = swarm._world('0.002', True, markers).read_text()
    for name, *_ in markers:
        assert text.count(f'<model name="{name}">') == 1
    assert text.count('<model name="arena_view">') == 1


def test_world_without_arena_view_has_no_markers(swarm):
    markers = [('ground_marker_iris_1', '1 0.8 0', 0, 0)]
    text = swarm._world('0.002', False, markers).read_text()
    assert 'ground_marker' not in text
