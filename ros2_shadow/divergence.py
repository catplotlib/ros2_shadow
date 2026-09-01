"""What the comparison engine produces."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class Severity(IntEnum):
    OK = 0
    WARNING = 1
    CRITICAL = 2


class Clock(str):
    """Which timestamp a pairing was made on.

    A node is not obliged to copy its input's stamp onto its output, and many
    do not, so a pairing may rest on arrival time instead. That weakens every
    number derived from it, and the difference is worth carrying around rather
    than hiding.
    """

    HEADER = "header.stamp"
    RECEIVE = "receive_time"


@dataclass
class MetricResult:
    name: str
    value: float
    severity: Severity = Severity.OK
    threshold: float | None = None
    detail: str = ""


@dataclass
class Comparison:
    """One matched pair of outputs and everything measured about it."""

    production_stamp: float
    candidate_stamp: float
    clock: str
    metrics: list[MetricResult] = field(default_factory=list)

    @property
    def pairing_error_s(self) -> float:
        return abs(self.production_stamp - self.candidate_stamp)

    @property
    def severity(self) -> Severity:
        return max((m.severity for m in self.metrics), default=Severity.OK)

    def worst(self) -> MetricResult | None:
        if not self.metrics:
            return None
        return max(self.metrics, key=lambda m: (m.severity, m.value))
