"""geometry_msgs/Twist.

Twist has no header, so pairing falls back to arrival time.
"""

from __future__ import annotations

import math

from ros2_shadow.metrics.base import register

TYPE = "geometry_msgs/msg/Twist"


def _linear(msg):
    return (msg.linear.x, msg.linear.y, msg.linear.z)


def _angular(msg):
    return (msg.angular.x, msg.angular.y, msg.angular.z)


def _norm(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


@register(TYPE, "linear_error")
def linear_error(production, candidate) -> float:
    return _norm(_linear(production), _linear(candidate))


@register(TYPE, "angular_error")
def angular_error(production, candidate) -> float:
    return _norm(_angular(production), _angular(candidate))


@register(TYPE, "linear_x_error")
def linear_x_error(production, candidate) -> float:
    return abs(production.linear.x - candidate.linear.x)


@register(TYPE, "direction_reversal")
def direction_reversal(production, candidate) -> float:
    """1.0 when the two commands drive opposite ways along x.

    Deliberately not a magnitude. A candidate asking for -0.3 where production
    asks for +0.4 is a different behaviour, not a slightly wrong number, and an
    error norm of 0.7 buries that. Small commands either side of zero are
    ignored, since sign is meaningless at a standstill.
    """
    p, c = production.linear.x, candidate.linear.x
    if abs(p) < 1e-3 or abs(c) < 1e-3:
        return 0.0
    return 1.0 if (p > 0) != (c > 0) else 0.0
