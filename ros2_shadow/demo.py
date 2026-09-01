"""Two publishers standing in for a production node and a candidate.

Lets the runtime be exercised without a simulator. The candidate tracks
production closely, then drifts, then briefly commands the opposite direction,
which is the case an error norm alone would understate.
"""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DemoPair(Node):
    def __init__(self, rate: float, drift_at: float, reverse_at: float):
        super().__init__("shadow_demo_pair")
        self.drift_at = drift_at
        self.reverse_at = reverse_at
        self.start = self.get_clock().now().nanoseconds * 1e-9

        self.production = self.create_publisher(Twist, "/planner/cmd_vel", 10)
        self.candidate = self.create_publisher(Twist, "/shadow/planner/cmd_vel", 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"publishing at {rate:g} Hz; candidate drifts at {drift_at:g}s, "
            f"reverses at {reverse_at:g}s"
        )

    def tick(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start

        production = Twist()
        production.linear.x = 0.4 + 0.05 * math.sin(t)
        production.angular.z = 0.2 * math.sin(t / 2.0)

        candidate = Twist()
        candidate.linear.x = production.linear.x
        candidate.angular.z = production.angular.z

        if t >= self.drift_at:
            candidate.linear.x += 0.06          # a steady, mild disagreement
            candidate.angular.z += 0.03
        if self.reverse_at <= t < self.reverse_at + 2.0:
            candidate.linear.x = -0.3           # briefly drives the other way

        self.production.publish(production)
        self.candidate.publish(candidate)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="shadow_demo_pair")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--drift-at", type=float, default=5.0)
    parser.add_argument("--reverse-at", type=float, default=12.0)
    args, _ = parser.parse_known_args(argv)

    rclpy.init()
    node = DemoPair(args.rate, args.drift_at, args.reverse_at)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


class DemoOne(Node):
    """A single publisher, for the isolation demo where production and the
    candidate are separate processes in separate domains."""

    def __init__(self, name: str, topic: str, offset: float, reverse_at: float):
        super().__init__(name)
        self.offset = offset
        self.reverse_at = reverse_at
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.publisher = self.create_publisher(Twist, topic, 10)
        # A candidate that also tries to drive the robot directly. In its own
        # domain this reaches nothing.
        self.hardware = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"publishing {topic} (offset {offset:+.2f})")

    def tick(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start
        msg = Twist()
        msg.linear.x = 0.4 + 0.05 * math.sin(t) + self.offset
        msg.angular.z = 0.2 * math.sin(t / 2.0)
        if self.reverse_at and self.reverse_at <= t < self.reverse_at + 2.0:
            msg.linear.x = -0.3
        self.publisher.publish(msg)
        if self.offset:
            self.hardware.publish(msg)


def _run_one(name, topic, offset, reverse_at, argv):
    parser = argparse.ArgumentParser(prog=name)
    parser.add_argument("--topic", default=topic)
    parser.add_argument("--offset", type=float, default=offset)
    parser.add_argument("--reverse-at", type=float, default=reverse_at)
    args, _ = parser.parse_known_args(argv)

    rclpy.init()
    node = DemoOne(name, args.topic, args.offset, args.reverse_at)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def production_main(argv=None):
    return _run_one("shadow_demo_production", "/planner/cmd_vel", 0.0, 0.0, argv)


def candidate_main(argv=None):
    return _run_one("shadow_demo_candidate", "/planner/cmd_vel", 0.06, 10.0, argv)
