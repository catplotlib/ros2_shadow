"""Shadow run configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# Topics a candidate must never publish on. A namespace already covers nodes
# that use relative names; this catches the ones that hardcode an absolute topic
# or build the name at runtime, and it reports rather than prevents.
DEFAULT_FORBIDDEN = [
    "/cmd_vel",
    "/joint_commands",
    "/joint_trajectory_controller/*",
    "/hardware/*",
    "/servo_node/*",
]


class ConfigError(ValueError):
    pass



@dataclass
class MetricSpec:
    name: str
    warning: float | None = None
    critical: float | None = None


@dataclass
class ShadowConfig:
    production_topic: str
    candidate_topic: str
    message_type: str
    metrics: list[MetricSpec] = field(default_factory=list)
    tolerance_ms: float = 20.0
    forbidden_topics: list[str] = field(default_factory=lambda: list(DEFAULT_FORBIDDEN))
    report_period_s: float = 2.0
    candidate_namespace: str = "/shadow"
    qos_reliability: str = "reliable"

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ShadowConfig":
        with Path(path).open() as f:
            return cls.from_dict(yaml.safe_load(f) or {})

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ShadowConfig":
        if not isinstance(data, dict):
            raise ConfigError("config must be a mapping at the top level")

        production = (data.get("production") or {}).get("topic")
        candidate = (data.get("shadow") or data.get("candidate") or {}).get("topic")
        comparison = data.get("comparison") or {}
        message_type = comparison.get("type")

        for label, value in (
            ("production.topic", production),
            ("shadow.topic", candidate),
            ("comparison.type", message_type),
        ):
            if not value:
                raise ConfigError(f"{label} is required")

        if production == candidate:
            raise ConfigError(
                "production.topic and shadow.topic are the same; the candidate "
                "would be compared against itself"
            )

        metrics = []
        for entry in comparison.get("metrics") or []:
            if isinstance(entry, str):
                metrics.append(MetricSpec(name=entry))
                continue
            if "name" not in entry:
                raise ConfigError(f"metric entry needs a name: {entry!r}")
            metrics.append(
                MetricSpec(
                    name=entry["name"],
                    warning=entry.get("warning"),
                    critical=entry.get("critical"),
                )
            )

        sync = comparison.get("synchronization") or {}
        safety = data.get("safety") or {}
        forbidden = safety.get("forbidden_topics", list(DEFAULT_FORBIDDEN))


        return cls(
            production_topic=production,
            candidate_topic=candidate,
            message_type=message_type,
            metrics=metrics,
            tolerance_ms=float(sync.get("tolerance_ms", 20.0)),
            forbidden_topics=forbidden,
            report_period_s=float((data.get("reporting") or {}).get("period_s", 2.0)),
            candidate_namespace=(data.get("shadow") or {}).get("namespace", "/shadow"),
            qos_reliability=comparison.get("qos", "reliable"),
        )
