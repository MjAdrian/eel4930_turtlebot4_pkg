#! /usr/bin/env python3
####################################
# The point of this file is to host an Interactive Marker Server. This server will
# host the individual waypoints that the robot should visit along its Nav2 path
# when patrolling an area.
#
# This interactive marker server can be used to update these waypoints from Rviz.
#
# For more information about Interactive Markers, please see:
# http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started (ROS1)
####################################

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker


class MarkerServer(Node):

    server: InteractiveMarkerServer | None

    def __init__(self):
        super().__init__("marker_server")
        self.server = None
        self.timer = self.create_timer(1, self.create_sphere)

    def set_server(self, server: InteractiveMarkerServer):
        self.server = server

    def create_sphere(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.scale = 0.2

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.name = "test name"
        if server is None:
            raise ValueError(
                "The interactive marker server has not yet been initialized."
            )

        server.insert(marker)
        server.applyChanges()


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = MarkerServer()

    server = InteractiveMarkerServer(node, "marker_server")

    server.applyChanges()

    node.set_server(server)
    rclpy.spin(node)
    server.shutdown()
