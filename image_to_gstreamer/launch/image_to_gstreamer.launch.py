from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_to_gstreamer',
            executable='image_to_gstreamer_node',
            name='image_to_gstreamer_node',
            parameters=[
                # Topics
                {'input_topic': '/cam/image_color'},

            ],
        )
    ])