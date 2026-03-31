import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gstreamer_to_ROS'),
        'config',
        'gstreamer_to_ROS.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='gstreamer_to_ROS',
                executable='gstreamer_to_ROS',
                name='gstreamer_to_ROS_node',
                parameters=[config],
                output='screen',
            )
        ]
    )
