import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('image_to_gstreamer'),
        'config',
        'stream.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='image_to_gstreamer',
                executable='image_to_gstreamer_node',
                name='image_to_gstreamer_node',
                parameters=[config],
                output='screen',
            ),
        ]
    )
