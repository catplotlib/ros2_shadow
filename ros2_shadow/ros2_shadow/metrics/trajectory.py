"""trajectory_msgs/JointTrajectory."""

from __future__ import annotations

import math

from ros2_shadow.metrics.base import register

TYPE = "trajectory_msgs/msg/JointTrajectory"


def _aligned(production, candidate):
    """Walk both trajectories joint by joint, in production's joint order.

    Joint order is not guaranteed to match between two planners, so comparing
    positions index by index would silently compare an elbow against a wrist.
    """
    index = {name: i for i, name in enumerate(candidate.joint_names)}
    shared = [(i, index[name]) for i, name in enumerate(production.joint_names) if name in index]

    for p_point, c_point in zip(production.points, candidate.points):
        for p_i, c_i in shared:
            if p_i < len(p_point.positions) and c_i < len(c_point.positions):
                yield p_point.positions[p_i], c_point.positions[c_i]


@register(TYPE, "joint_position_rmse")
def joint_position_rmse(production, candidate) -> float:
    diffs = [(a - b) ** 2 for a, b in _aligned(production, candidate)]
    if not diffs:
        return 0.0
    return math.sqrt(sum(diffs) / len(diffs))


@register(TYPE, "max_joint_delta")
def max_joint_delta(production, candidate) -> float:
    return max((abs(a - b) for a, b in _aligned(production, candidate)), default=0.0)


@register(TYPE, "endpoint_delta")
def endpoint_delta(production, candidate) -> float:
    """Difference at the final waypoint, which is where the arm ends up."""
    if not production.points or not candidate.points:
        return 0.0
    index = {name: i for i, name in enumerate(candidate.joint_names)}
    p_last, c_last = production.points[-1], candidate.points[-1]

    worst = 0.0
    for p_i, name in enumerate(production.joint_names):
        c_i = index.get(name)
        if c_i is None or p_i >= len(p_last.positions) or c_i >= len(c_last.positions):
            continue
        worst = max(worst, abs(p_last.positions[p_i] - c_last.positions[c_i]))
    return worst


@register(TYPE, "duration_delta")
def duration_delta(production, candidate) -> float:
    """Seconds between the two trajectories' planned completion times."""

    def end(traj):
        if not traj.points:
            return 0.0
        d = traj.points[-1].time_from_start
        return d.sec + d.nanosec * 1e-9

    return abs(end(production) - end(candidate))
