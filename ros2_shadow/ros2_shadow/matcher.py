"""Pairing production outputs with the candidate outputs that answer the same input.

This is the central problem. The two nodes see identical inputs but finish at
different times, so "latest against latest" compares unrelated results.

Ideally a node copies its input's stamp onto its output and pairing is exact.
Many nodes do not: Nav2's planner stamps its path with now(), not with the
stamp of the goal that produced it. So the field you want to join on is
frequently absent, and pairing falls back to arrival time, which is a weaker
claim. The clock used is carried on every pair rather than hidden.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

from ros2_shadow.divergence import Clock


@dataclass
class Pending:
    stamp: float          # the time the pairing is made on
    receive_time: float   # when this process saw it, always wall clock
    clock: str
    message: Any


def message_stamp(message: Any, receive_time: float) -> tuple[float, str]:
    """Prefer the message's own header stamp, fall back to arrival.

    A zero stamp means the publisher never populated the header, so it says
    nothing about ordering and must not be used for pairing.
    """
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if seconds > 0.0:
            return seconds, Clock.HEADER
    return receive_time, Clock.RECEIVE


class Matcher:
    def __init__(self, tolerance_s: float, max_wait_s: float | None = None):
        self.tolerance = tolerance_s
        # How long to hold an output before giving up on its partner. Declaring
        # an output unmatched the moment no partner is present would call every
        # slightly-late candidate a miss.
        self.max_wait = max_wait_s if max_wait_s is not None else max(tolerance_s * 5, 0.2)

        self._production: deque[Pending] = deque()
        self._candidate: deque[Pending] = deque()

        self.unmatched_production = 0
        self.unmatched_candidate = 0
        self.matched = 0

    def push_production(self, message: Any, receive_time: float) -> None:
        stamp, clock = message_stamp(message, receive_time)
        self._production.append(Pending(stamp, receive_time, clock, message))

    def push_candidate(self, message: Any, receive_time: float) -> None:
        stamp, clock = message_stamp(message, receive_time)
        self._candidate.append(Pending(stamp, receive_time, clock, message))

    def drain(self, now: float) -> list[tuple[Pending, Pending]]:
        """Emit every pair that can be settled, and retire outputs that waited
        too long for a partner."""
        pairs = []

        while self._production and self._candidate:
            production, candidate = self._production[0], self._candidate[0]
            delta = production.stamp - candidate.stamp

            # Timestamps carry nanosecond resolution, and seconds-as-float
            # cannot represent the boundary exactly: a 20 ms tolerance against
            # stamps 20 ms apart lands on 0.020000000000000018. Allow a
            # nanosecond so a pair exactly at the configured tolerance matches,
            # which is what the config says it does.
            if abs(delta) <= self.tolerance + 1e-9:
                self._production.popleft()
                self._candidate.popleft()
                pairs.append((production, candidate))
                self.matched += 1
                continue

            # The earlier of the two has no partner nearby. Its partner may
            # still be in flight, so only retire it once it has waited.
            earlier, buffer = (
                (production, self._production) if delta < 0 else (candidate, self._candidate)
            )
            if now - earlier.receive_time <= self.max_wait:
                break
            buffer.popleft()
            if delta < 0:
                self.unmatched_production += 1
            else:
                self.unmatched_candidate += 1

        self._expire(self._production, now, production_side=True)
        self._expire(self._candidate, now, production_side=False)
        return pairs

    def _expire(self, buffer: deque[Pending], now: float, production_side: bool) -> None:
        while buffer and now - buffer[0].receive_time > self.max_wait:
            buffer.popleft()
            if production_side:
                self.unmatched_production += 1
            else:
                self.unmatched_candidate += 1

    @property
    def pending(self) -> int:
        return len(self._production) + len(self._candidate)
