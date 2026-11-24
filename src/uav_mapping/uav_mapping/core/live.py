"""The `live` provider: ask a language model for the regions (no ROS).

Same shape as the replay provider (`provider(clue, world) -> raw text`), so
`make_prior` treats both alike: parse strictly, one retry, else uniform.

What it sends is the landmark file, the map bounds and the clue, and nothing
else: no target positions, no drone state. What comes back is text for
`parse_regions`, and nothing else of it is used (the hard rule).

The key comes only from the environment variable `LLM_API_KEY`, never from a
file or a parameter, so it cannot be committed.

No `temperature` is sent. The roadmap asks for temperature 0, but current
Claude models accept only their default sampling and answer a request that
sets it with a 400. Repeatability comes from replay instead: a live answer is
frozen as a recording (`save_recording`) and every later run reads that.
"""

import os
from typing import Any, List, Optional

from .landmarks import Landmark, World
from .prior import MAX_REGIONS, ProviderUnavailable

KEY_VARIABLE = 'LLM_API_KEY'
MAX_TOKENS = 16000          # room for thinking as well as the JSON

SYSTEM = f"""You turn a search-and-rescue clue into where searchers should look first.

You are given a map: its bounds and named places, in metres (x east, y north). \
You are given one clue written by an operator. Reply with ONE JSON object and \
nothing else (no prose, no code fence):

{{"regions": [
  {{"shape": "circle", "center": [x, y], "radius": r, "weight": w, "confidence": c}},
  {{"shape": "capsule", "start": [x, y], "end": [x, y], "radius": r, "weight": w, "confidence": c}}
]}}

Rules:
- One region per place the clue points at, at most {MAX_REGIONS}. Use a circle \
for a point or an area, a capsule (everything within radius of a line segment) \
for a path or a stretch of a line.
- Centre every region on the named place's coordinates from the map; use a \
radius in metres that fits how precisely the clue says where.
- weight in [0, 1]: how strongly the clue points at this place (1 = it names \
it directly). confidence in [0, 1]: how sure you are you read the clue and \
the map correctly.
- Use only places on the map. If the clue names no place on the map, reply \
{{"regions": []}}.
- Every number is a plain JSON number."""


def _describe(lm: Landmark) -> str:
    pts = ', '.join(f"({x:g}, {y:g})" for x, y in lm.points)
    text = f"- {lm.name} ({lm.type}): {pts}"
    return f"{text} - {lm.description}" if lm.description else text


def build_prompt(clue: str, world: World) -> str:
    """The user message: the map, then the clue."""
    x0, y0, x1, y1 = world.bounds
    lines: List[str] = [
        f"Map '{world.name}': x from {x0:g} to {x1:g}, y from {y0:g} to {y1:g}.",
        'Places:']
    lines += [_describe(lm) for lm in world.landmarks]
    lines += ['', f"Clue: {clue}"]
    return '\n'.join(lines)


class LiveProvider:
    """Asks the Claude API. ``outputs`` keeps every raw answer in order, so a
    caller can freeze the one that parsed (the last, since make_prior stops at
    the first valid one)."""

    name = 'live'

    def __init__(self, model: str, client: Optional[Any] = None,
                 environ=os.environ):
        if not model:
            raise ProviderUnavailable("the live provider needs a model name")
        self.model = model
        self._client = client
        self._environ = environ
        self.outputs: List[str] = []

    def _get_client(self):
        if self._client is None:
            key = self._environ.get(KEY_VARIABLE, '')
            if not key:
                raise ProviderUnavailable(f"{KEY_VARIABLE} is not set")
            try:
                import anthropic
            except ImportError:
                raise ProviderUnavailable(
                    "the 'anthropic' package is not installed "
                    "(pip install anthropic)") from None
            self._client = anthropic.Anthropic(api_key=key)
        return self._client

    def __call__(self, clue: str, world: World) -> str:
        client = self._get_client()
        response = client.messages.create(
            model=self.model, max_tokens=MAX_TOKENS, system=SYSTEM,
            messages=[{'role': 'user', 'content': build_prompt(clue, world)}])
        if response.stop_reason == 'refusal':
            raise RuntimeError('the model declined to answer')
        text = ''.join(b.text for b in response.content
                       if getattr(b, 'type', '') == 'text')
        self.outputs.append(text)
        return text
