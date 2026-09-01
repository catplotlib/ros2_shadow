"""Metric registry, keyed by message type.

Different outputs need completely different comparisons. An RMSE over a Twist
says almost nothing; a sign flip on linear.x says the robot is about to reverse.
"""

from ros2_shadow.metrics.base import METRICS, Metric, register, metrics_for
from ros2_shadow.metrics import pose, trajectory, twist  # noqa: F401  (registers them)

__all__ = ["METRICS", "Metric", "register", "metrics_for"]
