"""geometry_msgs/PoseStamped."""

from __future__ import annotations

import math

from ros2_shadow.metrics.base import register

TYPE = "geometry_msgs/msg/PoseStamped"


@register(TYPE, "translation_error")
def translation_error(production, candidate) -> float:
    a, b = production.pose.position, candidate.pose.position
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


@register(TYPE, "rotation_error")
def rotation_error(production, candidate) -> float:
    """Angle between the two orientations, in radians.

    q and -q describe the same rotation, so the dot product is taken absolute
    before the arccos; without that, equivalent orientations report pi.
    """
    a, b = production.pose.orientation, candidate.pose.orientation
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return 2.0 * math.acos(min(1.0, dot))
