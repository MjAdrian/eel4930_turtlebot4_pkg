#! /usr/bin/env python3
from __future__ import annotations
import sys

import rclpy
from threading import Thread
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from autoro_interfaces.srv import Path
from rclpy.node import Node
from rclpy.duration import Duration


class AutoRoNavigator(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 0
        )
        self.pose_client = self.create_client(Path, "/request_patrol_path")
        self.msg = None

    def odom_callback(self, msg):
        self.msg = msg

    def request_pose_list(self) -> Path.Response | None:
        while not self.pose_client.wait_for_service(timeout_sec = 1):
            self.get_logger().warn("Patrol path request service is not available.")
        future = self.pose_client.call_async(Path.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init(args=sys.argv)
    navigator = AutoRoNavigator()
    init_pose = PoseStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.position.z = 0.0
    init_pose.pose.orientation.w = 0.72
    init_pose.pose.orientation.z = -0.68
    navigator.setInitialPose(init_pose)

    navigator.waitUntilNav2Active()

    autoro_share = get_package_share_directory("autoro_navigation")
    navigator.changeMap(f"{autoro_share}/maps/maec_map_edited.yaml")

    stamps = []
    poses = navigator.request_pose_list()
    if poses is None:
        exit(1)
    for pose in poses.path.poses:
        stamps.append(PoseStamped(header = Header(frame_id = "map"), pose = pose))
    navigator.goThroughPoses(stamps)

    print(stamps[0])
    print(stamps)

    i = 0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        i += 1
        if feedback and i % 5 == 0:
            sec = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimated time of arrival to step {i}: {sec:.2f}s")

    result = navigator.getResult()
    print(result)

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == "__main__":
    main()
