from __future__ import annotations

from typing import Any, Callable

# {message type: {metric name: callable(production, candidate) -> float}}
METRICS: dict[str, dict[str, Callable[[Any, Any], float]]] = {}

Metric = Callable[[Any, Any], float]


def register(message_type: str, name: str):
    def wrap(fn: Metric) -> Metric:
        METRICS.setdefault(message_type, {})[name] = fn
        return fn

    return wrap


def metrics_for(message_type: str) -> dict[str, Metric]:
    return METRICS.get(message_type, {})
