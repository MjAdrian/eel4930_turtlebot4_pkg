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
import rich

import rclpy
import random
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl


class MarkerServer(Node):

    server: InteractiveMarkerServer | None

    def __init__(self):
        super().__init__("marker_server")
        self.server = None
        self.timer = self.create_timer(30, self.create_sphere)

    def set_server(self, server: InteractiveMarkerServer):
        self.server = server
        self.create_sphere()

    def process_feedback(self, feedback):
        print(f"received {type(feedback)}")

    def create_sphere(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "map"
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.name = "test name"

        viz_marker = Marker()
        viz_marker.type = Marker.SPHERE
        viz_marker.scale.x = 0.3
        viz_marker.scale.y = 0.3
        viz_marker.scale.z = 0.3
        viz_marker.color.r = random.random()
        viz_marker.color.g = random.random()
        viz_marker.color.b = random.random()
        viz_marker.color.a = 1.0

        control_marker = InteractiveMarkerControl()
        control_marker.always_visible = True
        control_marker.markers.append(viz_marker)
        marker.controls.append(control_marker)

        movement_marker = InteractiveMarkerControl()
        movement_marker.name = "move_xy"
        movement_marker.orientation.w = 1.0
        movement_marker.orientation.x = 0.0
        movement_marker.orientation.y = 1.0
        movement_marker.orientation.z = 0.0
        movement_marker.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        movement_marker.always_visible = True
        marker.controls.append(movement_marker)

        if server is None:
            raise ValueError(
                "The interactive marker server has not yet been initialized."
            )

        server.insert(marker, feedback_callback = self.process_feedback)
        server.applyChanges()
        rich.print(f"Just inserted marker with {len(marker.controls)} controls...")
        rich.print(marker.controls)


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = MarkerServer()

    server = InteractiveMarkerServer(node, "marker_server")

    server.applyChanges()

    node.set_server(server)
    rclpy.spin(node)
    server.shutdown()
