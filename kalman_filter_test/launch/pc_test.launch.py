from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='tb4_movement',
        #     executable='tb4_movement',
        #     name='tb4_movement',
        #     output='screen'
        # ),
        Node(
            package='tb4_movement',
            executable='static_aiming',
            name='static_aiming',
            output='screen'
        ),
        Node(
            package='kalman_filter_test',
            executable='node_circle_test',
            name='node_circle_test',
            output='screen'
        ),
    ])
