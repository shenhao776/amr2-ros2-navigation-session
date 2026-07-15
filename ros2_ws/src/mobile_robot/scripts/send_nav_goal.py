#!/usr/bin/env python3
"""Send an interactive x/y/yaw goal to Nav2."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def finite_float(prompt: str) -> float:
    """Read one finite floating-point value from the terminal."""
    while True:
        raw_value = input(prompt).strip()
        try:
            value = float(raw_value)
        except ValueError:
            print("  Invalid number. Example inputs: 2, -1.5, 90")
            continue
        if not math.isfinite(value):
            print("  Please enter a finite number.")
            continue
        return value


def interactive_goal() -> tuple[float, float, float]:
    print("\nEnter a target in the ROS 'map' coordinate frame.")
    print("x and y are in metres.")
    print("yaw is in degrees: 0 = +X, 90 = +Y, -90 = -Y.\n")

    x = finite_float("Target x [m] (example: 2.0): ")
    y = finite_float("Target y [m] (example: -1.5): ")
    yaw_deg = finite_float("Target yaw [degrees] (example: 90): ")
    return x, y, yaw_deg


class NavGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("interactive_nav_goal_client")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._last_feedback_time = 0.0

    def wait_for_nav2(self) -> None:
        print("Waiting for the Nav2 'navigate_to_pose' action server...")
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=2.0):
            print("  Nav2 is not ready yet. Keep start_full_system.launch.py running.")

    def feedback_callback(self, feedback_msg) -> None:
        now = time.monotonic()
        if now - self._last_feedback_time < 1.0:
            return
        self._last_feedback_time = now
        distance = feedback_msg.feedback.distance_remaining
        print(f"  Distance remaining: {distance:.2f} m")

    def send_goal(self, x: float, y: float, yaw_deg: float) -> int:
        self.wait_for_nav2()
        if not rclpy.ok():
            return 1

        yaw_rad = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        print(f"Sending goal: x={x:.3f} m, y={y:.3f} m, yaw={yaw_deg:.1f}°")
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            print("Goal rejected by Nav2.")
            return 2

        print("Goal accepted. Press Ctrl+C to cancel.")
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print("\nCancel requested; waiting for Nav2...")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            print("Navigation goal cancelled.")
            return 130

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Navigation succeeded: the robot reached the goal.")
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            print("Navigation was cancelled.")
            return 3
        if status == GoalStatus.STATUS_ABORTED:
            print("Navigation was aborted. Check the Nav2 launch terminal.")
            return 4

        print(f"Navigation finished with status code {status}.")
        return 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send an x/y/yaw goal to the Nav2 navigate_to_pose action."
    )
    parser.add_argument("--x", type=float, help="target x in metres")
    parser.add_argument("--y", type=float, help="target y in metres")
    parser.add_argument("--yaw-deg", type=float, help="target yaw in degrees")
    args = parser.parse_args()

    supplied = (args.x, args.y, args.yaw_deg)
    if any(value is not None for value in supplied) and not all(
        value is not None for value in supplied
    ):
        parser.error("provide --x, --y, and --yaw-deg together, or omit all three")
    return args


def main() -> int:
    args = parse_args()
    if args.x is None:
        try:
            x, y, yaw_deg = interactive_goal()
        except (EOFError, KeyboardInterrupt):
            print("\nNo goal sent.")
            return 130
    else:
        x, y, yaw_deg = args.x, args.y, args.yaw_deg

    if not all(math.isfinite(value) for value in (x, y, yaw_deg)):
        print("x, y, and yaw must be finite numbers.", file=sys.stderr)
        return 2

    answer = input(
        f"Send x={x:.3f} m, y={y:.3f} m, yaw={yaw_deg:.1f}°? [y/N]: "
    ).strip().lower()
    if answer not in ("y", "yes"):
        print("No goal sent.")
        return 0

    rclpy.init()
    node = NavGoalClient()
    try:
        return node.send_goal(x, y, yaw_deg)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

