from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {"dev": "/dev/input/js0"},
                {"deadzone": 0.1},
                {"autorepeat_rate": 20.0},
            ],
        ),
        # Launch the custom joy_movement node
        Node(
            package='movement_test',
            executable='joy_movement',
            name='joy_movement',
        ),
    ])
