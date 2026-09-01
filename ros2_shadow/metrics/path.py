"""nav_msgs/Path.

Two planners that route around an obstacle on opposite sides produce paths that
are numerically far apart and may both be perfectly good. Distance alone cannot
tell you which case you are in, so several shapes of difference are reported
rather than one number:

  where it ends up      endpoint_distance
  how far it strays     hausdorff_distance, mean_deviation
  how much work it is   length_difference

A large Hausdorff with a small endpoint distance is a different route to the
same place. A small Hausdorff with a large endpoint distance is the same route
stopping somewhere else. Those are different problems.
"""

from __future__ import annotations

import numpy as np

from ros2_shadow.metrics.base import register

TYPE = "nav_msgs/msg/Path"


# Returned when a comparison has no data to work with. The node reports these
# as warnings rather than folding them into the statistics, so an empty
# candidate output cannot look like a candidate that agreed.
UNMEASURABLE = float("nan")


def _points(path) -> np.ndarray:
    """Path as an (n, 2) array of xy. Planar, because clearance and route
    divergence are ground-plane questions."""
    poses = getattr(path, "poses", None) or []
    if not poses:
        return np.empty((0, 2))
    return np.array([[p.pose.position.x, p.pose.position.y] for p in poses], dtype=float)


def _pairwise(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.linalg.norm(a[:, None, :] - b[None, :, :], axis=-1)


def _length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


@register(TYPE, "endpoint_distance")
def endpoint_distance(production, candidate) -> float:
    a, b = _points(production), _points(candidate)
    if not len(a) or not len(b):
        return UNMEASURABLE
    return float(np.linalg.norm(a[-1] - b[-1]))


@register(TYPE, "start_distance")
def start_distance(production, candidate) -> float:
    a, b = _points(production), _points(candidate)
    if not len(a) or not len(b):
        return UNMEASURABLE
    return float(np.linalg.norm(a[0] - b[0]))


@register(TYPE, "hausdorff_distance")
def hausdorff_distance(production, candidate) -> float:
    """The worst excursion either path makes from the other.

    Symmetric on purpose: a candidate that follows production exactly and then
    adds a detour is not the same as one that skips a section, and the one-way
    distance misses one of those.
    """
    a, b = _points(production), _points(candidate)
    if not len(a) or not len(b):
        return UNMEASURABLE
    distances = _pairwise(a, b)
    return float(max(distances.min(axis=1).max(), distances.min(axis=0).max()))


@register(TYPE, "mean_deviation")
def mean_deviation(production, candidate) -> float:
    """Average distance from each candidate point to the production path.

    Less alarmist than Hausdorff: one spike moves the maximum a long way but
    barely moves this.
    """
    a, b = _points(production), _points(candidate)
    if not len(a) or not len(b):
        return UNMEASURABLE
    return float(_pairwise(b, a).min(axis=1).mean())


@register(TYPE, "length_difference")
def length_difference(production, candidate) -> float:
    return abs(_length(_points(production)) - _length(_points(candidate)))
