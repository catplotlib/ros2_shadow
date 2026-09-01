"""Drives two Nav2 planner servers with identical goals.

Both servers hold the same map and the same costmap settings and differ only in
the planning algorithm, so any divergence between their paths is the algorithm
and nothing else. Each result is republished on a plain topic for the shadow
comparison to pick up.

The start pose is sent explicitly rather than read from TF, so the planners can
be exercised without a robot or a simulator running.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Points in the warehouse map's largest connected free region, each with at
# least 0.8 m of clearance so they survive costmap inflation. Chosen by
# distance transform over the occupancy grid rather than by eye; picking
# plausible-looking coordinates gave goals in occupied space and both planners
# correctly returned nothing.
START = (-8.47, 8.99)
GOALS = [(-12.22, -19.90), (-9.16, -2.86), (0.71, 11.24), (14.12, 18.44)]


def pose(x: float, y: float, frame: str = "map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


class Nav2Probe(Node):
    def __init__(self, period: float = 3.0):
        super().__init__("nav2_probe")
        self.index = 0

        self.planners = {
            "production": ActionClient(self, ComputePathToPose, "/production/compute_path_to_pose"),
            "candidate": ActionClient(self, ComputePathToPose, "/candidate/compute_path_to_pose"),
        }
        self.path_pubs = {
            "production": self.create_publisher(Path, "/planner/path", 10),
            "candidate": self.create_publisher(Path, "/shadow/planner/path", 10),
        }
        self.create_timer(period, self.request_pair)
        self.get_logger().info("waiting for both planner servers")

    def request_pair(self) -> None:
        for name, client in self.planners.items():
            if not client.server_is_ready():
                self.get_logger().warning(f"{name} planner not ready yet", once=True)
                return

        goal = ComputePathToPose.Goal()
        goal.start = pose(*START)
        goal.goal = pose(*GOALS[self.index % len(GOALS)])
        goal.use_start = True
        self.index += 1

        stamp = self.get_clock().now().to_msg()
        for name, client in self.planners.items():
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda f, name=name, stamp=stamp: self._accepted(f, name, stamp))

    def _accepted(self, future, name: str, stamp) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(f"{name} rejected the goal")
            return
        handle.get_result_async().add_done_callback(
            lambda f, name=name, stamp=stamp: self._result(f, name, stamp)
        )

    def _result(self, future, name: str, stamp) -> None:
        path = future.result().result.path
        if not path.poses:
            self.get_logger().warning(f"{name} returned an empty path")
        # Both planners answered the same request, so both outputs carry the
        # stamp of that request. This is the pairing key the matcher wants and
        # the case a planner restamping with now() would deny it.
        path.header.stamp = stamp
        path.header.frame_id = "map"
        self.path_pubs[name].publish(path)
        self.get_logger().info(f"{name}: {len(path.poses)} poses", throttle_duration_sec=5.0)


def main(argv=None) -> int:
    rclpy.init()
    node = Nav2Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
