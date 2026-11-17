"""Target positions out of Gazebo's scene description (no ROS).

`gz service -s /world/<w>/scene/info --reqtype gz.msgs.Empty --reptype
gz.msgs.Scene` prints the whole world as protobuf text. We need only the
top-level models and their world pose, so this walks the braces instead of
pulling in a protobuf parser (the container has no gz Python bindings).
Zero fields are omitted by protobuf, so a model at the origin has no x or y.
"""


def parse_scene_models(text):
    """{model name: (x, y, z)} for every top-level model in a Scene dump."""
    models = {}
    stack = []
    name = None
    pos = {}
    for raw in text.splitlines():
        line = raw.strip()
        if line.endswith('{'):
            stack.append(line[:-1].strip())
            if stack == ['model']:
                name, pos = None, {}
        elif line == '}':
            if stack == ['model'] and name is not None:
                models[name] = (pos.get('x', 0.0), pos.get('y', 0.0),
                                pos.get('z', 0.0))
            if stack:
                stack.pop()
        elif ':' in line:
            key, _, value = line.partition(':')
            value = value.strip()
            if stack == ['model'] and key == 'name':
                name = value.strip('"')
            elif stack == ['model', 'pose', 'position'] and key in ('x', 'y', 'z'):
                pos[key] = float(value)
    return models


def select_targets(models, prefix='target_'):
    """[(name, x, y)] for models whose name starts with prefix, sorted."""
    return sorted((n, p[0], p[1]) for n, p in models.items()
                  if n.startswith(prefix))
