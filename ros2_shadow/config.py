"""Shadow run configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# Topics a candidate must never publish on. Namespace remapping is cooperative,
# so this is a backstop and a clear error message, not the isolation mechanism.
# Real isolation is a separate ROS_DOMAIN_ID; see docs/isolation.md.
DEFAULT_FORBIDDEN = [
    "/cmd_vel",
    "/joint_commands",
    "/joint_trajectory_controller/*",
    "/hardware/*",
    "/servo_node/*",
]


class ConfigError(ValueError):
    pass


def _forbidden(topic: str, patterns: list[str]) -> bool:
    from fnmatch import fnmatchcase

    return any(topic == p or fnmatchcase(topic, p) for p in patterns)


@dataclass
class MetricSpec:
    name: str
    warning: float | None = None
    critical: float | None = None


@dataclass
class BridgedTopic:
    topic: str
    type: str
    republish_as: str | None = None


@dataclass
class IsolationConfig:
    """Which topics cross the domain boundary, and in which direction.

    Everything not listed here cannot cross at all. That is the whole point:
    the candidate runs in a DDS domain where the hardware topics do not exist,
    so reaching them is not a policy it can violate.
    """

    production_domain: int = 0
    candidate_domain: int = 42
    inputs: list[BridgedTopic] = field(default_factory=list)
    outputs: list[BridgedTopic] = field(default_factory=list)


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
    isolation: "IsolationConfig | None" = None

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

        isolation = None
        iso = data.get("isolation")
        if iso:
            def bridged(entries, direction):
                out = []
                for entry in entries or []:
                    if "topic" not in entry or "type" not in entry:
                        raise ConfigError(
                            f"isolation.{direction} entries need 'topic' and 'type': {entry!r}"
                        )
                    target = entry.get("republish_as", entry["topic"])
                    # A topic coming back from the candidate domain must never
                    # land on something that drives hardware.
                    if direction == "outputs" and _forbidden(target, forbidden):
                        raise ConfigError(
                            f"isolation.outputs would republish onto {target}, which is "
                            "listed as forbidden; the candidate would reach hardware"
                        )
                    out.append(BridgedTopic(entry["topic"], entry["type"], target))
                return out

            if iso.get("production_domain") == iso.get("candidate_domain"):
                raise ConfigError(
                    "isolation.production_domain and candidate_domain are the same; "
                    "there would be no isolation"
                )
            isolation = IsolationConfig(
                production_domain=int(iso.get("production_domain", 0)),
                candidate_domain=int(iso.get("candidate_domain", 42)),
                inputs=bridged(iso.get("inputs"), "inputs"),
                outputs=bridged(iso.get("outputs"), "outputs"),
            )

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
            isolation=isolation,
        )
