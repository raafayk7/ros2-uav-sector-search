"""The detector's error model, and one frame of detections (no ROS).

A ground-truth detector knows exactly where every target is. To stand in for
YOLO it has to be wrong in the ways YOLO is wrong, so downstream code (the
heatmap, the D32 confirmation rule) is tested against realistic evidence:

  miss         a visible target is not reported this frame
  pixel noise  a reported target lands a few pixels off
  false alarm  a box appears where there is nothing, at a random pixel

False alarms are independent per frame, so one rarely repeats in the same
sector. A false positive that stays on one spot (a misleading landmark) is a
separate item in the roadmap and is not modelled here.
"""

from dataclasses import dataclass

from .projection import in_frame, project_ground_point


@dataclass(frozen=True)
class NoiseModel:
    p_miss: float = 0.20            # per visible target per frame
    p_false: float = 0.05           # per frame: chance of one false box
    pixel_sigma: float = 5.0        # px, Gaussian on u and v
    hit_score: tuple = (0.60, 0.95)  # uniform range for a true detection
    false_score: tuple = (0.30, 0.70)


@dataclass(frozen=True)
class Detection:
    u: float
    v: float
    size_px: float      # box side, cosmetic
    score: float
    is_true: bool       # for counting and tests only: consumers never see it


def frame_detections(targets_xy, cam_pos, world_R_cam, intrinsics, size,
                     noise, rng, marker_size=1.0):
    """One camera frame: the detections to publish (possibly none).

    targets_xy   iterable of (x, y) ground positions in the arena frame
    intrinsics   (fx, fy, cx, cy)
    size         (width, height) in px
    rng          numpy Generator, so a seed reproduces a run

    The footprint check uses the true pixel, before noise: a target outside
    the image is never reported. A noisy pixel that lands outside is dropped
    too, since a real detector cannot box something off the image.
    """
    fx, fy, cx, cy = intrinsics
    width, height = size
    out = []
    for x, y in targets_xy:
        proj = project_ground_point(x, y, cam_pos, world_R_cam, fx, fy, cx, cy)
        if proj is None or not in_frame(proj[0], proj[1], width, height):
            continue
        if rng.random() < noise.p_miss:
            continue
        u = proj[0] + rng.normal(0.0, noise.pixel_sigma)
        v = proj[1] + rng.normal(0.0, noise.pixel_sigma)
        if not in_frame(u, v, width, height):
            continue
        out.append(Detection(u, v, marker_size * fx / proj[2],
                             rng.uniform(*noise.hit_score), True))
    if rng.random() < noise.p_false:
        out.append(Detection(rng.uniform(0.0, width), rng.uniform(0.0, height),
                             40.0, rng.uniform(*noise.false_score), False))
    return out
