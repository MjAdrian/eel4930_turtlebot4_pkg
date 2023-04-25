from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_lite_test',
            executable='oakd_pub_test',
            name='oakd_pub_test',
            output='screen'
        ),
        # Node(
        #     package='oakd_lite_test',
        #     executable='oakd_sub_test',
        #     name='oakd_sub_test',
        #     output='screen'
        # )
    ])
