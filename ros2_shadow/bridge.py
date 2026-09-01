"""The wall.

A candidate that shares a DDS domain with production can always reach hardware.
Namespace remapping is cooperative, and a graph watcher only reports the breach
after a message is already on the wire.

So the candidate gets its own domain. In that domain the hardware topics do not
exist, and nothing it publishes can reach anything real. This process is the
only route between the two, and it carries exactly the topics named in the
config: inputs in, candidate outputs back out under names this process chooses.

Two rclpy contexts run in one process, one per domain. A context can only see
its own domain, so a subscription here cannot accidentally observe the other
side.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from ros2_shadow.config import IsolationConfig, ShadowConfig


def _qos(reliability: str) -> QoSProfile:
    return QoSProfile(
        depth=20,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=(
            QoSReliabilityPolicy.BEST_EFFORT
            if reliability == "best_effort"
            else QoSReliabilityPolicy.RELIABLE
        ),
    )


class DomainBridge:
    def __init__(self, isolation: IsolationConfig, reliability: str = "reliable"):
        self.isolation = isolation
        self.qos = _qos(reliability)

        self.production_context = rclpy.Context()
        rclpy.init(context=self.production_context, domain_id=isolation.production_domain)
        self.candidate_context = rclpy.Context()
        rclpy.init(context=self.candidate_context, domain_id=isolation.candidate_domain)

        self.production_node = Node(
            "shadow_bridge_production", context=self.production_context)
        self.candidate_node = Node(
            "shadow_bridge_candidate", context=self.candidate_context)

        self.forwarded = {"inputs": 0, "outputs": 0}
        self._wire()

    def _wire(self) -> None:
        # Inputs: production domain -> candidate domain. The candidate only
        # ever receives on these; nothing it does travels back along them.
        for entry in self.isolation.inputs:
            message_class = get_message(entry.type)
            target = entry.republish_as or entry.topic
            publisher = self.candidate_node.create_publisher(message_class, target, self.qos)

            def forward_in(msg, publisher=publisher):
                publisher.publish(msg)
                self.forwarded["inputs"] += 1

            self.production_node.create_subscription(
                message_class, entry.topic, forward_in, self.qos)

        # Outputs: candidate domain -> production domain, under names this
        # process chooses. Config loading has already refused any target that
        # matches a forbidden pattern.
        for entry in self.isolation.outputs:
            message_class = get_message(entry.type)
            target = entry.republish_as or entry.topic
            publisher = self.production_node.create_publisher(message_class, target, self.qos)

            def forward_out(msg, publisher=publisher):
                publisher.publish(msg)
                self.forwarded["outputs"] += 1

            self.candidate_node.create_subscription(
                message_class, entry.topic, forward_out, self.qos)

    def manifest(self) -> str:
        iso = self.isolation
        lines = [
            "",
            f"ros2_shadow bridge   production domain {iso.production_domain}"
            f"  <->  candidate domain {iso.candidate_domain}",
            "",
            "  into the candidate domain:",
        ]
        lines += [
            f"    {e.topic}  ->  {e.republish_as or e.topic}   ({e.type})"
            for e in iso.inputs
        ] or ["    (nothing)"]
        lines.append("")
        lines.append("  back into the production domain:")
        lines += [
            f"    {e.topic}  ->  {e.republish_as or e.topic}   ({e.type})"
            for e in iso.outputs
        ] or ["    (nothing)"]
        lines += [
            "",
            "  Nothing else crosses. Anything the candidate publishes on a topic",
            "  not listed above stays inside its own domain.",
            "",
        ]
        return "\n".join(lines)

    def spin(self) -> None:
        executors = []
        for context, node in (
            (self.production_context, self.production_node),
            (self.candidate_context, self.candidate_node),
        ):
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)

        threads = [threading.Thread(target=e.spin, daemon=True) for e in executors[1:]]
        for thread in threads:
            thread.start()
        try:
            executors[0].spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            for executor in executors:
                executor.shutdown()

    def shutdown(self) -> None:
        self.production_node.destroy_node()
        self.candidate_node.destroy_node()
        rclpy.try_shutdown(context=self.production_context)
        rclpy.try_shutdown(context=self.candidate_context)


def run(config: ShadowConfig) -> int:
    if config.isolation is None:
        print(
            "shadow: this config has no 'isolation' section, so there is nothing "
            "to bridge and the candidate is not isolated",
        )
        return 2

    bridge = DomainBridge(config.isolation, config.qos_reliability)
    print(bridge.manifest(), flush=True)
    try:
        bridge.spin()
    finally:
        print(
            f"forwarded {bridge.forwarded['inputs']} input messages, "
            f"{bridge.forwarded['outputs']} candidate outputs",
            flush=True,
        )
        bridge.shutdown()
    return 0
