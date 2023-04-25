from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[
                {"enable_depth": True},
                {"enable_color": False},
                {"enable_infra1": False},
                {"enable_infra2": False},
                {"depth_width": 640},
                {"depth_height": 480},
                {"depth_fps": 30},
            ]
        )
    ])
