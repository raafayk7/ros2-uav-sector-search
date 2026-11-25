"""The `live` provider: ask a language model for the regions (no ROS).

Same shape as the replay provider (`provider(clue, world) -> raw text`), so
`make_prior` treats both alike: parse strictly, one retry, else uniform.

What it sends is the landmark file, the map bounds and the clue, and nothing
else: no target positions, no drone state. What comes back is text for
`parse_regions`, and nothing else of it is used (the hard rule).

The model's name picks the vendor (gemini-* or claude-*), and with it the one
key it may use: GEMINI_API_KEY or ANTHROPIC_API_KEY (D51). So a key only ever
goes to its own vendor, and there is no setting that can disagree with the
model a recording names. Keys come only from the environment (normally filled
from the gitignored docker/llm.env), never from a parameter.

No `temperature` is sent: current Claude models answer a request that sets it
with a 400, and Gemini ignores it from 3.5 Flash-Lite and 3.6 Flash on. The
roadmap's temperature 0 is replaced by replay: a live answer is frozen as a
recording (`save_recording`) and every later run reads that.
"""

import os
from typing import Any, Callable, Dict, List, NamedTuple, Optional, Tuple

from .landmarks import Landmark, World
from .prior import MAX_REGIONS, ProviderUnavailable

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


# ---- Vendors ----------------------------------------------------------------

class Answer(NamedTuple):
    text: str
    usage: Dict[str, int]       # tokens: input, output, thinking (billed as output)


class Vendor(NamedTuple):
    prefix: str                                     # of the model's name
    key_variable: str
    package: str                                    # what pip calls it
    make_client: Callable[[str], Any]               # key -> client
    ask: Callable[[Any, str, str], Answer]          # client, model, prompt


def _gemini_client(key: str):
    from google import genai
    return genai.Client(api_key=key, vertexai=False)


def _ask_gemini(client, model: str, prompt: str) -> Answer:
    response = client.models.generate_content(
        model=model, contents=prompt,
        config={'system_instruction': SYSTEM, 'max_output_tokens': MAX_TOKENS,
                'response_mime_type': 'application/json'})
    if not response.candidates:
        feedback = response.prompt_feedback
        raise RuntimeError('the model declined to answer '
                           f"({feedback and feedback.block_reason})")
    reason = response.candidates[0].finish_reason
    if getattr(reason, 'name', reason) != 'STOP':
        raise RuntimeError(f"the model stopped early ({reason})")
    u = response.usage_metadata
    return Answer(response.text or '', {
        'input': u.prompt_token_count or 0,
        'output': u.candidates_token_count or 0,
        'thinking': u.thoughts_token_count or 0} if u else {})


def _claude_client(key: str):
    import anthropic
    return anthropic.Anthropic(api_key=key)


def _ask_claude(client, model: str, prompt: str) -> Answer:
    response = client.messages.create(
        model=model, max_tokens=MAX_TOKENS, system=SYSTEM,
        messages=[{'role': 'user', 'content': prompt}])
    if response.stop_reason == 'refusal':
        raise RuntimeError('the model declined to answer')
    if response.stop_reason != 'end_turn':
        raise RuntimeError(f"the model stopped early ({response.stop_reason})")
    text = ''.join(b.text for b in response.content
                   if getattr(b, 'type', '') == 'text')
    u = getattr(response, 'usage', None)
    return Answer(text, {'input': u.input_tokens,
                         'output': u.output_tokens} if u else {})


VENDORS: Tuple[Vendor, ...] = (
    Vendor('gemini-', 'GEMINI_API_KEY', 'google-genai', _gemini_client, _ask_gemini),
    Vendor('claude-', 'ANTHROPIC_API_KEY', 'anthropic', _claude_client, _ask_claude),
)


def vendor_of(model: str) -> Vendor:
    for vendor in VENDORS:
        if model.startswith(vendor.prefix):
            return vendor
    raise ProviderUnavailable(
        f"unknown model {model!r}: the name must start with "
        + ' or '.join(v.prefix for v in VENDORS))


class LiveProvider:
    """Asks the model. ``outputs`` keeps every raw answer in order, so a caller
    can freeze the one that parsed (the last, since make_prior stops at the
    first valid one); ``usage`` keeps each answer's token counts."""

    name = 'live'

    def __init__(self, model: str, client: Optional[Any] = None,
                 environ=os.environ):
        if not model:
            raise ProviderUnavailable("the live provider needs a model name")
        self.model = model
        self.vendor = vendor_of(model)
        self._client = client
        self._environ = environ
        self.outputs: List[str] = []
        self.usage: List[Dict[str, int]] = []

    def _key(self) -> str:
        return self._environ.get(self.vendor.key_variable, '')

    def _get_client(self):
        if self._client is None:
            key, variable = self._key(), self.vendor.key_variable
            if not key:
                raise ProviderUnavailable(f"{variable} is not set")
            if any(c.isspace() or c in '"\'' for c in key):
                raise ProviderUnavailable(
                    f"{variable} holds a quote or a space; docker's --env-file "
                    "keeps them as part of the value")
            try:
                self._client = self.vendor.make_client(key)
            except ImportError:
                raise ProviderUnavailable(
                    f"the {self.vendor.package!r} package is not installed "
                    "(docker/requirements-llm.txt)") from None
        return self._client

    def __call__(self, clue: str, world: World) -> str:
        client = self._get_client()
        try:
            answer = self.vendor.ask(client, self.model, build_prompt(clue, world))
        except Exception as e:      # noqa: BLE001 - warnings are logged: no key in them
            key = self._key()
            if not key or key not in str(e):
                raise
            raise RuntimeError(str(e).replace(key, '<key>')) from None
        self.outputs.append(answer.text)
        self.usage.append(answer.usage)
        return answer.text
