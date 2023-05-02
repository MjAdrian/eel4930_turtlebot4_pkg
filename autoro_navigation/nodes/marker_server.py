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
import rclpy.logging
import rich
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

logger = rclpy.logging.get_logger(__name__)


class MarkerServer(Node):

    server: InteractiveMarkerServer | None
    colors: dict[str, list[int]]  # name to rgb
    poses: dict[str, Pose]

    def __init__(self):
        super().__init__("marker_server")
        self.server = None
        self.colors = {
            "red": [255, 173, 173],
            "orange": [255, 214, 165],
            "yellow": [253, 255, 182],
            "green": [202, 255, 191],
            "aqua": [155, 246, 255],
            "blue": [160, 196, 255],
            "purple": [189, 178, 255],
            "pink": [255, 198, 255],
        }
        self.poses = {}
        self.point_pub = self.create_publisher(PoseArray, "/patrol_path", 0)

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
            points_tups = [(p[0], p[1]) for name, p in points.items()]
            for i, tup in enumerate(points_tups):
                self.create_sphere(tup[0], tup[1], i, list(self.colors.values())[i])

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        position = [
            feedback.pose.position.x,
            feedback.pose.position.y,
        ]
        orientation = [
            feedback.pose.orientation.w,
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
        ]
        logger.log(
            f"{feedback.marker_name} is now at {position} and {orientation}",
            rclpy.logging.LoggingSeverity.INFO,
        )
        self.poses[feedback.marker_name] = feedback.pose
        self.point_pub.publish(PoseArray(poses=list(self.poses.values())))

    def create_sphere(
        self, pos: list[float], orientation: list[float], position: int, rgb: list[int]
    ) -> None:
        uniq_id = f"marker_{position}"
        name = f"step_{position}"
        arrow_scale = 0.5
        text_scale = 0.3

        marker = InteractiveMarker()
        marker.header.frame_id = "map"
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = float(orientation[0])
        marker.pose.orientation.x = float(orientation[1])
        marker.pose.orientation.y = float(orientation[2])
        marker.pose.orientation.z = float(orientation[3])
        marker.name = uniq_id
        # marker.description = position

        viz_marker = Marker()
        viz_marker.type = Marker.ARROW
        viz_marker.scale.x = arrow_scale
        viz_marker.scale.y = arrow_scale * 0.25
        viz_marker.scale.z = arrow_scale * 0.25
        viz_marker.color.r = rgb[0] / 255.0
        viz_marker.color.g = rgb[1] / 255.0
        viz_marker.color.b = rgb[2] / 255.0
        viz_marker.pose.position.x = 0.0  # float(x)
        viz_marker.pose.position.y = 0.0  # float(y)
        viz_marker.pose.position.z = 0.0
        viz_marker.color.a = 1.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.orientation.w = 1.0
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 1.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.position.x = 1.0
        text_marker.text = name
        text_marker.scale.x = 0.3
        text_marker.scale.y = 0.3
        text_marker.scale.z = text_scale
        text_marker.color.r = 0.5
        text_marker.color.g = 0.5
        text_marker.color.b = 0.5
        text_marker.color.a = 1.0

        control_marker = InteractiveMarkerControl()
        control_marker.always_visible = True
        control_marker.markers.append(viz_marker)
        control_marker.markers.append(text_marker)
        marker.controls.append(control_marker)

        movement_marker = InteractiveMarkerControl()
        movement_marker.name = uniq_id
        movement_marker.orientation.w = 1.0
        movement_marker.orientation.x = 0.0
        movement_marker.orientation.y = 1.0
        movement_marker.orientation.z = 0.0
        movement_marker.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        movement_marker.always_visible = True
        marker.controls.append(movement_marker)

        if server is None:
            raise ValueError(
                "The interactive marker server has not yet been initialized."
            )

        self.poses[uniq_id] = viz_marker.pose
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
