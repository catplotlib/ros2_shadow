"""The shadow runtime: subscribe to both outputs, pair them, measure, report."""

from __future__ import annotations

import statistics
from collections import defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from ros2_shadow import safety
from ros2_shadow.config import ShadowConfig
from ros2_shadow.divergence import Comparison, MetricResult, Severity
from ros2_shadow.matcher import Matcher
from ros2_shadow.metrics import metrics_for

_LEVEL = {
    Severity.OK: DiagnosticStatus.OK,
    Severity.WARNING: DiagnosticStatus.WARN,
    Severity.CRITICAL: DiagnosticStatus.ERROR,
}


class ShadowNode(Node):
    def __init__(self, config: ShadowConfig):
        super().__init__("ros2_shadow")
        self.config = config

        message_class = get_message(config.message_type)
        available = metrics_for(config.message_type)
        if not available:
            raise SystemExit(
                f"no metrics registered for {config.message_type}; "
                f"known types: {', '.join(sorted(set(m for m in _known_types())))}"
            )

        self._selected = {}
        for spec in config.metrics:
            if spec.name not in available:
                raise SystemExit(
                    f"unknown metric '{spec.name}' for {config.message_type}; "
                    f"available: {', '.join(sorted(available))}"
                )
            self._selected[spec.name] = (available[spec.name], spec)

        self.matcher = Matcher(tolerance_s=config.tolerance_ms / 1000.0)
        self.samples: dict[str, list[float]] = defaultdict(list)
        self.counts = {Severity.WARNING: 0, Severity.CRITICAL: 0}
        self.last_critical: Comparison | None = None
        # (metric, severity) -> (last logged at, events suppressed since)
        self._log_state: dict[tuple[str, int], tuple[float, int]] = {}
        self.clock_used: set[str] = set()
        self.suspended = False

        qos = QoSProfile(
            depth=20,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
                if config.qos_reliability == "best_effort"
                else QoSReliabilityPolicy.RELIABLE
            ),
        )

        self.create_subscription(
            message_class, config.production_topic,
            lambda m: self.matcher.push_production(m, self._now()), qos)
        self.create_subscription(
            message_class, config.candidate_topic,
            lambda m: self.matcher.push_candidate(m, self._now()), qos)

        self.divergence_pub = self.create_publisher(DiagnosticArray, "/shadow/divergence", 10)

        self.create_timer(0.02, self._compare)
        self.create_timer(config.report_period_s, self.print_report)
        self.create_timer(0.5, self._check_safety)

        self.get_logger().info(
            f"comparing candidate {config.candidate_topic} against production "
            f"{config.production_topic} ({config.message_type})"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- comparison -------------------------------------------------------

    def _compare(self) -> None:
        if self.suspended:
            return
        for production, candidate in self.matcher.drain(self._now()):
            self.clock_used.add(production.clock)
            comparison = Comparison(
                production_stamp=production.stamp,
                candidate_stamp=candidate.stamp,
                clock=production.clock,
            )

            for name, (fn, spec) in self._selected.items():
                try:
                    value = float(fn(production.message, candidate.message))
                except Exception as exc:  # a malformed message must not kill the run
                    self.get_logger().warning(f"metric {name} failed: {exc}")
                    continue

                severity = Severity.OK
                threshold = None
                if spec.critical is not None and value >= spec.critical:
                    severity, threshold = Severity.CRITICAL, spec.critical
                elif spec.warning is not None and value >= spec.warning:
                    severity, threshold = Severity.WARNING, spec.warning

                comparison.metrics.append(MetricResult(name, value, severity, threshold))
                self.samples[name].append(value)

            if comparison.severity != Severity.OK:
                self.counts[comparison.severity] += 1
                if comparison.severity == Severity.CRITICAL:
                    self.last_critical = comparison
                self._publish(comparison)
                self._log(comparison)

    def _publish(self, comparison: Comparison) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for metric in comparison.metrics:
            if metric.severity == Severity.OK:
                continue
            status = DiagnosticStatus()
            status.level = _LEVEL[metric.severity]
            status.name = f"shadow/{metric.name}"
            status.hardware_id = self.config.candidate_topic
            status.message = (
                f"{metric.name} {metric.value:.4f} over threshold {metric.threshold}"
            )
            status.values = [
                KeyValue(key="value", value=f"{metric.value:.6f}"),
                KeyValue(key="threshold", value=str(metric.threshold)),
                KeyValue(key="production_stamp", value=f"{comparison.production_stamp:.6f}"),
                KeyValue(key="candidate_stamp", value=f"{comparison.candidate_stamp:.6f}"),
                KeyValue(key="pairing_error_ms", value=f"{comparison.pairing_error_s * 1e3:.2f}"),
                KeyValue(key="clock", value=comparison.clock),
            ]
            message.status.append(status)
        self.divergence_pub.publish(message)

    LOG_INTERVAL_S = 2.0

    def _log(self, comparison: Comparison) -> None:
        """Log the first occurrence, then at most one line per interval.

        A sustained divergence produces one event per message. Logging each one
        turns a 20 Hz disagreement into hundreds of identical lines and hides
        everything else, so repeats are counted and folded into the next line.
        """
        worst = comparison.worst()
        if worst is None:
            return

        now = self._now()
        key = (worst.name, int(worst.severity))
        last_at, suppressed = self._log_state.get(key, (None, 0))

        if last_at is not None and now - last_at < self.LOG_INTERVAL_S:
            self._log_state[key] = (last_at, suppressed + 1)
            return

        line = (
            f"{worst.severity.name}: {worst.name} = {worst.value:.4f} "
            f"(threshold {worst.threshold}) at t={comparison.production_stamp:.3f}"
        )
        if suppressed:
            line += f"  [+{suppressed} more since last line]"

        if worst.severity == Severity.CRITICAL:
            self.get_logger().error(line)
        else:
            self.get_logger().warning(line)
        self._log_state[key] = (now, 0)

    # -- safety -----------------------------------------------------------

    def _check_safety(self) -> None:
        if self.suspended:
            return
        offences = safety.scan(self, self.config.forbidden_topics, self.config.candidate_namespace)
        if not offences:
            return
        self.suspended = True
        for topic, node_name in offences:
            self.get_logger().fatal(
                f"candidate node {node_name} is publishing on forbidden topic {topic}"
            )
        self.get_logger().fatal("shadow comparison suspended; the candidate can reach hardware")

    # -- reporting --------------------------------------------------------

    def print_report(self) -> None:
        print(self.render_report(), flush=True)

    def render_report(self) -> str:
        m = self.matcher
        clocks = ", ".join(sorted(self.clock_used)) or "none yet"
        lines = [
            "",
            f"ros2_shadow  {self.config.candidate_topic} vs {self.config.production_topic}",
            f"  matched {m.matched}   unmatched prod {m.unmatched_production}   "
            f"unmatched cand {m.unmatched_candidate}   pending {m.pending}",
            f"  paired on {clocks}",
        ]

        if self.samples:
            lines.append("")
            lines.append(f"  {'metric':<24}{'mean':>10}{'p95':>10}{'max':>10}")
            for name in sorted(self.samples):
                values = self.samples[name]
                ordered = sorted(values)
                p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
                lines.append(
                    f"  {name:<24}{statistics.fmean(values):>10.4f}{p95:>10.4f}{max(values):>10.4f}"
                )

        lines.append("")
        lines.append(
            f"  warnings {self.counts[Severity.WARNING]}   "
            f"critical {self.counts[Severity.CRITICAL]}"
        )
        if self.last_critical is not None:
            worst = self.last_critical.worst()
            lines.append(
                f"  last critical: {worst.name} = {worst.value:.4f} "
                f"at t={self.last_critical.production_stamp:.3f}"
            )
        if self.suspended:
            lines.append("  SUSPENDED: candidate published on a forbidden topic")
        return "\n".join(lines)


def _known_types():
    from ros2_shadow.metrics.base import METRICS

    return METRICS.keys()


def run(config: ShadowConfig) -> int:
    rclpy.init()
    node = ShadowNode(config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(node.render_report(), flush=True)
        failed = node.counts[Severity.CRITICAL] > 0 or node.suspended
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failed else 0
