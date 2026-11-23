"""Invariants of the CI files, the Docker files and the packaged skill.

Nothing else tests these, and each rule here was a comment someone had to
remember: the workflows share values with the Dockerfile and the compose file,
the smoke workflow must never run for pull requests, and the .skill bundle must
match SKILL.md. They read repo files only, so no ROS is needed.
"""

import re
import subprocess
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CI_YML = ROOT / '.github' / 'workflows' / 'ci.yml'
SMOKE_YML = ROOT / '.github' / 'workflows' / 'smoke.yml'
DOCKERFILE = ROOT / 'docker' / 'Dockerfile'
COMPOSE = ROOT / 'docker' / 'docker-compose.yml'
SKILL_SRC = ROOT / 'skills' / 'multi-agent-uavs' / 'SKILL.md'
SKILL_BUNDLE = ROOT / 'skills' / 'multi-agent-uavs.skill'

yaml = pytest.importorskip('yaml')
needs_repo = pytest.mark.skipif(
    not CI_YML.exists(), reason='run from a checkout that has .github/')


def _triggers(path):
    doc = yaml.safe_load(path.read_text())
    # YAML 1.1 reads a bare `on` as True
    return doc.get('on', doc.get(True))


@needs_repo
def test_workflows_parse():
    for path in (CI_YML, SMOKE_YML):
        assert 'jobs' in yaml.safe_load(path.read_text()), path.name


@needs_repo
def test_smoke_workflow_never_runs_for_pull_requests():
    # A self-hosted runner on a public repo runs whatever a fork's PR contains.
    triggers = _triggers(SMOKE_YML)
    assert 'pull_request' not in triggers
    assert 'pull_request_target' not in triggers
    assert set(triggers) <= {'workflow_dispatch', 'push'}


@needs_repo
def test_ci_builds_ardupilot_msgs_at_the_dockerfiles_commit():
    pinned = re.search(r'^ARG ARDUPILOT_REF=(\w+)', DOCKERFILE.read_text(), re.M)
    in_ci = yaml.safe_load(CI_YML.read_text())['env']['ARDUPILOT_REF']
    assert pinned and in_ci == pinned.group(1)


@needs_repo
def test_smoke_workflow_uses_the_composes_dds_participant_limit():
    wanted = re.search(r'CYCLONEDDS_URI=(<CycloneDDS>.*</CycloneDDS>)',
                       COMPOSE.read_text())
    used = re.search(r'CYCLONEDDS_URI=(<CycloneDDS>.*</CycloneDDS>)',
                     SMOKE_YML.read_text())
    assert wanted and used and wanted.group(1) == used.group(1)


@needs_repo
def test_ci_scripts_are_valid_and_executable():
    scripts = sorted((ROOT / 'ci').glob('*.sh'))
    assert scripts, 'no ci/*.sh found'
    for script in scripts:
        assert script.stat().st_mode & 0o111, f'{script.name} is not executable'
        done = subprocess.run(['bash', '-n', str(script)],
                              capture_output=True, text=True)
        assert done.returncode == 0, f'{script.name}: {done.stderr}'
    checker = ROOT / 'ci' / 'smoke_check.py'
    compile(checker.read_text(), str(checker), 'exec')


@pytest.mark.skipif(not SKILL_BUNDLE.exists(), reason='no skill bundle here')
def test_skill_bundle_matches_its_source():
    # CLAUDE.md: a stale bundle silently ships an old skill; run skills/pack.sh.
    with zipfile.ZipFile(SKILL_BUNDLE) as bundle:
        names = [n for n in bundle.namelist() if n.endswith('SKILL.md')]
        assert len(names) == 1, names
        assert bundle.read(names[0]) == SKILL_SRC.read_bytes(), \
            'skills/multi-agent-uavs.skill is stale: run skills/pack.sh'
