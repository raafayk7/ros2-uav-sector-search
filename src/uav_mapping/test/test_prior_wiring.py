"""Session 17: the `live` provider, the prior as a message, recordings saved
from a live answer, the hard rule for the new code, and belief_map_node
waiting for the prior (needs ROS sourced; skipped otherwise)."""

import json
import re
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from uav_mapping.core import landmarks as lmk
from uav_mapping.core.belief import BeliefMap
from uav_mapping.core.geometry import Grid
from uav_mapping.core import live as live_module
from uav_mapping.core.live import (SYSTEM, VENDORS, LiveProvider, build_prompt,
                                   vendor_of)
from uav_mapping.core.prior import (ProviderUnavailable, load_replay,
                                    live_recording, make_prior, parse_recordings,
                                    parse_regions,
                                    prior_message, read_prior_message,
                                    save_recording)

GRID = Grid()
ROOT = Path(__file__).resolve().parents[3]
WORLD_FILE = ROOT / 'worlds/runway/landmarks.yaml'
RECORDINGS = ROOT / 'worlds/runway/recordings.json'
HUT = 'One hiker, last seen near the hut.'
HUT_JSON = next(r.output for r in parse_recordings(RECORDINGS.read_text())
                if r.clue == HUT)


@pytest.fixture(scope='module')
def world():
    return lmk.load_world(WORLD_FILE)


class FakeClaude:
    """Stands in for anthropic.Anthropic: answers from a list, keeps the calls."""

    model, variable = 'claude-x', 'ANTHROPIC_API_KEY'
    refused, cut_off = 'refusal', 'max_tokens'

    def __init__(self, *texts, stop='end_turn', error=None):
        self._texts, self._stop, self._error = list(texts), stop, error
        self.calls = []
        self.messages = self

    def create(self, **kwargs):
        self.calls.append(kwargs)
        if self._error:
            raise self._error
        return SimpleNamespace(
            stop_reason=self._stop,
            usage=SimpleNamespace(input_tokens=500, output_tokens=60),
            content=[SimpleNamespace(type='thinking', thinking=''),
                     SimpleNamespace(type='text', text=self._texts.pop(0))])


class FakeGemini:
    """Stands in for google.genai.Client."""

    model, variable = 'gemini-x', 'GEMINI_API_KEY'
    refused, cut_off = None, 'MAX_TOKENS'       # refused: no candidates at all

    def __init__(self, *texts, stop='STOP', error=None):
        self._texts, self._stop, self._error = list(texts), stop, error
        self.calls = []
        self.models = self

    def generate_content(self, **kwargs):
        self.calls.append(kwargs)
        if self._error:
            raise self._error
        reason = SimpleNamespace(name=self._stop)       # an enum in the SDK
        return SimpleNamespace(
            text=self._texts.pop(0),
            candidates=[SimpleNamespace(finish_reason=reason)] if self._stop else [],
            prompt_feedback=SimpleNamespace(block_reason='SAFETY'),
            usage_metadata=SimpleNamespace(
                prompt_token_count=500, candidates_token_count=60,
                thoughts_token_count=None))


@pytest.fixture(params=[FakeClaude, FakeGemini], ids=['claude', 'gemini'])
def fake(request):
    return request.param


# ---- The prompt -------------------------------------------------------------

def test_the_prompt_is_the_map_and_the_clue(world):
    text = build_prompt(HUT, world)
    assert 'x from 0 to 40, y from 0 to 40' in text
    for lm in world.landmarks:
        assert lm.name in text
    assert '(34, 7)' in text and 'a stone shelter' in text
    assert text.endswith(f"Clue: {HUT}")


def test_the_prompt_says_nothing_about_targets_or_drones(world):
    text = (SYSTEM + build_prompt(HUT, world)).lower()
    for word in ('target', 'drone', 'uav', 'sector', 'follower'):
        assert word not in text


# ---- The provider -----------------------------------------------------------

def test_a_live_answer_becomes_the_same_prior_as_the_recording(world, fake):
    live = LiveProvider(fake.model, fake(HUT_JSON))
    got = make_prior(HUT, world, GRID, live)
    want = make_prior(HUT, world, GRID, load_replay(RECORDINGS))
    assert not got.failed and np.array_equal(got.start, want.start)
    assert live.outputs == [HUT_JSON]
    (usage,) = live.usage
    assert usage['input'] == 500 and usage['output'] == 60


def test_the_request_is_the_model_the_rules_and_the_prompt(world, fake):
    client = fake(HUT_JSON)
    LiveProvider(fake.model, client, environ={})(HUT, world)
    (call,) = client.calls
    assert call['model'] == fake.model
    text = repr(call)
    assert repr(SYSTEM)[1:-1] in text and repr(build_prompt(HUT, world))[1:-1] in text
    for word in ('temperature', 'top_p', 'top_k', 'tools'):
        assert f"'{word}'" not in text


def test_gemini_is_asked_for_json(world):
    client = FakeGemini(HUT_JSON)
    LiveProvider('gemini-x', client)(HUT, world)
    assert client.calls[0]['config']['response_mime_type'] == 'application/json'


def test_an_invalid_answer_is_retried_once_and_the_last_output_is_the_valid_one(
        world, fake):
    live = LiveProvider(fake.model, fake('not json', HUT_JSON))
    prior = make_prior(HUT, world, GRID, live)
    assert not prior.failed and len(live.outputs) == 2
    assert parse_regions(live.outputs[-1])


def test_two_invalid_answers_fly_uniform_with_a_warning(world, fake):
    prior = make_prior(HUT, world, GRID,
                       LiveProvider(fake.model, fake('nope', '{}')))
    assert prior.failed and np.all(prior.start == 0.5)
    assert len(prior.warnings) == 3         # two attempts, then the uniform note


@pytest.mark.parametrize('how', ['refused', 'cut_off'])
def test_a_refusal_or_a_cut_off_answer_is_a_failed_attempt(world, fake, how):
    live = LiveProvider(fake.model, fake(HUT_JSON, HUT_JSON, stop=getattr(fake, how)))
    prior = make_prior(HUT, world, GRID, live)
    assert prior.failed and live.outputs == []
    assert {'refused': 'declined', 'cut_off': 'stopped early'}[how] in prior.warnings[0]


def test_no_key_is_one_attempt_and_a_warning_naming_the_variable(world, fake):
    live = LiveProvider(fake.model, environ={})
    prior = make_prior(HUT, world, GRID, live)
    assert prior.failed and np.all(prior.start == 0.5)
    assert len(prior.warnings) == 2 and fake.variable in prior.warnings[0]
    with pytest.raises(ProviderUnavailable):
        make_prior(HUT, world, GRID, live, strict=True)


def test_one_vendors_key_is_never_used_for_the_other(world, fake):
    other = {v.key_variable: 'k' for v in VENDORS if v.key_variable != fake.variable}
    other['LLM_API_KEY'] = other['GOOGLE_API_KEY'] = 'k'
    with pytest.raises(ProviderUnavailable, match=fake.variable):
        LiveProvider(fake.model, environ=other)(HUT, world)


def test_only_the_right_key_reaches_the_client(world, fake, monkeypatch):
    got = []
    monkeypatch.setattr(live_module, 'VENDORS', tuple(
        v._replace(make_client=lambda key: got.append(key) or fake(HUT_JSON))
        for v in VENDORS))
    environ = {v.key_variable: f'key-of-{v.prefix}' for v in VENDORS}
    LiveProvider(fake.model, environ=environ)(HUT, world)
    assert got == ['key-of-' + fake.model[:-1]]


@pytest.mark.parametrize('key', ['"abc"', "'abc'", ' abc', 'abc\r'])
def test_a_key_with_quotes_or_spaces_is_refused_without_showing_it(world, fake, key):
    with pytest.raises(ProviderUnavailable) as e:
        LiveProvider(fake.model, environ={fake.variable: key})(HUT, world)
    assert 'abc' not in str(e.value)


def test_a_key_in_an_error_is_not_logged(world, fake):
    live = LiveProvider(fake.model, fake(error=ValueError('bad header s3cret!')),
                        environ={fake.variable: 's3cret'})
    prior = make_prior(HUT, world, GRID, live)
    assert prior.failed and 'bad header <key>' in prior.warnings[0]
    assert 's3cret' not in ''.join(prior.warnings)


def test_a_missing_package_is_one_attempt_naming_it(world, fake, monkeypatch):
    def missing(key):
        raise ImportError
    monkeypatch.setattr(live_module, 'VENDORS', tuple(
        v._replace(make_client=missing) for v in VENDORS))
    prior = make_prior(HUT, world, GRID,
                       LiveProvider(fake.model, environ={fake.variable: 'k'}))
    assert len(prior.warnings) == 2
    assert vendor_of(fake.model).package in prior.warnings[0]


@pytest.mark.parametrize('model', ['', 'gpt-x', 'my-gemini-x', 'Claude-x'])
def test_a_model_without_a_known_vendor_is_unavailable_not_a_crash(model):
    with pytest.raises(ProviderUnavailable):
        LiveProvider(model)


def test_the_keys_are_read_from_the_environment_only():
    src = (ROOT / 'src/uav_mapping/uav_mapping/core/live.py').read_text()
    assert re.findall(r"environ\.get\(([\w.]+)", src) == ['self.vendor.key_variable']
    assert 'open(' not in src
    assert [v.key_variable for v in VENDORS] == ['GEMINI_API_KEY', 'ANTHROPIC_API_KEY']


def test_the_key_file_is_ignored_by_git_and_docker_and_the_example_is_empty():
    assert 'docker/llm.env*' in (ROOT / '.gitignore').read_text().splitlines()
    assert 'llm.env*' in (ROOT / 'docker/.dockerignore').read_text().splitlines()
    example = (ROOT / 'docker/llm.env.example').read_text()
    keys = [l for l in example.splitlines() if l and not l.startswith('#')]
    assert keys == [f'{v.key_variable}=' for v in VENDORS]


# ---- The prior as a message -------------------------------------------------

def test_a_prior_message_round_trips(world):
    prior = make_prior(HUT, world, GRID, load_replay(RECORDINGS))
    start, failed = read_prior_message(prior_message(prior, HUT, 'replay'), GRID)
    assert not failed and np.allclose(start, prior.start)
    data = json.loads(prior_message(prior, HUT, 'replay'))
    assert data['clue'] == HUT and data['provider'] == 'replay'
    assert data['regions'][0]['shape'] == 'circle'


@pytest.mark.parametrize('text', ['', 'nope', '{}', '{"start": 1, "failed": false}',
                                  '{"start": [[0.5]], "failed": false}'])
def test_an_unreadable_prior_message_is_an_error(text):
    with pytest.raises(ValueError):
        read_prior_message(text, GRID)


# ---- Freezing a live answer -------------------------------------------------

def test_a_saved_live_recording_replays_and_replaces(world, tmp_path):
    path = tmp_path / 'recordings.json'
    rec = live_recording(world, HUT, 'some-model', HUT_JSON, '2026-12-01')
    assert save_recording(path, rec) is False
    again = live_recording(world, HUT.upper(), 'some-model', HUT_JSON, '2026-12-02')
    assert save_recording(path, again) is True      # same clue, case aside
    replay = load_replay(path, require_live=True)    # the eval would accept it
    assert replay(HUT, world) == HUT_JSON
    assert path.read_text().endswith('\n')


def test_a_broken_recordings_file_is_not_overwritten(world, tmp_path):
    path = tmp_path / 'recordings.json'
    path.write_text('{"recordings": [{"world": "x"}]}')
    with pytest.raises(ValueError):
        save_recording(path, live_recording(world, HUT, 'm', HUT_JSON, 'd'))
    assert path.read_text() == '{"recordings": [{"world": "x"}]}'


# ---- The hard rule ----------------------------------------------------------

def test_the_new_prior_code_cannot_reach_a_drone():
    pkg = ROOT / 'src/uav_mapping/uav_mapping'
    live = (pkg / 'core/live.py').read_text()
    for word in ('rclpy', '_msgs', 'cmd_gps_pose', 'uav_planning'):
        assert word not in live, f"live.py mentions {word}"
    node = (pkg / 'mission_prior_node.py').read_text()
    for word in ('cmd_gps_pose', 'uav_planning', 'ardupilot', 'geometry_msgs',
                 'create_client', 'create_subscription', 'mode_switch'):
        assert word not in node, f"mission_prior_node.py mentions {word}"
    assert re.findall(r'^from (\w+_msgs)\.', node, re.M) == ['std_msgs']
    assert node.count('create_publisher') == 1


# ---- belief_map_node waits for the prior ------------------------------------

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def ros():
    """Run belief_map_node in this process, with a helper node to talk to it."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    state = {}

    def start(*params):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args, domain_id=77)
        from uav_mapping.belief_map_node import BeliefMapNode
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node = BeliefMapNode()
        helper = rclpy.create_node('test_helper')
        state.update(
            node=node, helper=helper, heatmaps=[],
            prior=helper.create_publisher(String, '/mission_prior', latched),
            pose=helper.create_publisher(PoseStamped, '/iris_1/pose', 10),
            ex=SingleThreadedExecutor())
        helper.create_subscription(OccupancyGrid, '/heatmap',
                                   lambda m: state['heatmaps'].append(list(m.data)),
                                   latched)
        state['ex'].add_node(node)
        state['ex'].add_node(helper)
        return state

    def spin(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not (until and until()):
            state['ex'].spin_once(timeout_sec=0.05)

    def send_pose():
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.pose.position.z = 8.0
        msg.pose.orientation.w = 1.0
        state['pose'].publish(msg)

    state['start'], state['spin'], state['send_pose'] = start, spin, send_pose
    yield state
    if 'node' in state:
        state['ex'].shutdown()
        state['node'].destroy_node()
        state['helper'].destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_without_a_clue_the_map_is_built_at_once(ros):
    ros['start']()
    assert ros['node'].map is not None


def test_with_a_clue_nothing_is_published_until_the_prior_arrives(ros, world):
    ros['start']('wait_for_prior:=true')
    node = ros['node']
    assert node.map is None
    ros['send_pose']()
    ros['spin'](1.0)
    assert ros['heatmaps'] == []                # silent, not an all-unknown map
    from std_msgs.msg import String
    prior = make_prior(HUT, world, GRID, load_replay(RECORDINGS))
    ros['prior'].publish(String(data=prior_message(prior, HUT, 'replay')))
    ros['spin'](5.0, until=lambda: node.map is not None)
    assert node.map is not None
    ros['send_pose']()
    ros['spin'](5.0, until=lambda: ros['heatmaps'])
    col, row = GRID.cell_of(34, 7)
    assert ros['heatmaps'][-1][row * 8 + col] == 74


def test_a_failed_prior_gives_todays_map(ros, world):
    ros['start']('wait_for_prior:=true')
    from std_msgs.msg import String
    from uav_mapping.core.prior import uniform_prior
    ros['prior'].publish(String(data=prior_message(
        uniform_prior(GRID, ['no recording']), HUT, 'replay')))
    ros['spin'](5.0, until=lambda: ros['node'].map is not None)
    assert np.array_equal(ros['node'].map.occupancy_data(),
                          BeliefMap(GRID).occupancy_data())


def test_no_prior_in_time_flies_uniform(ros):
    ros['start']('wait_for_prior:=true', 'prior_timeout:=0.5')
    ros['spin'](5.0, until=lambda: ros['node'].map is not None)
    assert ros['node'].map is not None
    assert np.array_equal(ros['node'].map.occupancy_data(),
                          BeliefMap(GRID).occupancy_data())


def test_an_unreadable_prior_flies_uniform(ros):
    ros['start']('wait_for_prior:=true')
    from std_msgs.msg import String
    ros['prior'].publish(String(data='garbage'))
    ros['spin'](5.0, until=lambda: ros['node'].map is not None)
    assert ros['node'].map is not None
