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

import random
import sys

import rclpy
import rich
import yaml
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from ament_index_python.packages import get_package_share_directory


class MarkerServer(Node):

    server: InteractiveMarkerServer | None

    def __init__(self):
        super().__init__("marker_server")
        self.server = None

    def set_server(self, server: InteractiveMarkerServer):
        self.server = server
        self.read_points()

    def read_points(self) -> None:
        autoro_share = get_package_share_directory("autoro_navigation")
        with open(f"{autoro_share}/config/patrol_path.yaml", "r") as f:
            file_contents = yaml.safe_load(f)
            if "points" not in file_contents:
                raise ValueError(
                    "Patrol path configuration file does not contain list of path points to make a path."
                )
            points = file_contents["points"]
            for point in points:
                self.create_sphere(point[0], point[1])

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        print(f"received {feedback.marker_name}, {feedback.control_name}, {feedback.pose.position.x}, {feedback.pose.position.y}")

    def create_sphere(self, x: float, y: float):
        marker = InteractiveMarker()
        marker.header.frame_id = "map"
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.description = f"{x}, {y}"

        viz_marker = Marker()
        viz_marker.type = Marker.SPHERE
        viz_marker.scale.x = 0.3
        viz_marker.scale.y = 0.3
        viz_marker.scale.z = 0.3
        viz_marker.color.r = random.random()
        viz_marker.color.g = random.random()
        viz_marker.color.b = random.random()
        viz_marker.pose.position.x = float(x)
        viz_marker.pose.position.y = float(y)
        viz_marker.pose.position.z = 0.0
        viz_marker.color.a = 1.0

        control_marker = InteractiveMarkerControl()
        control_marker.always_visible = True
        control_marker.markers.append(viz_marker)
        marker.controls.append(control_marker)

        movement_marker = InteractiveMarkerControl()
        movement_marker.name = f"move_xy_{x}{y}"
        movement_marker.orientation.w = 1.0
        movement_marker.orientation.x = 0.0
        movement_marker.orientation.y = 1.0
        movement_marker.orientation.z = 0.0
        movement_marker.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        movement_marker.always_visible = True
        movement_marker.markers.append(viz_marker)
        marker.controls.append(movement_marker)

        if server is None:
            raise ValueError(
                "The interactive marker server has not yet been initialized."
            )

        server.insert(marker, feedback_callback=self.process_feedback)
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
