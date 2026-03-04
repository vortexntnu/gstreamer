from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='127.0.0.1',
        description='Destination host for UDP stream'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5000',
        description='Destination UDP port'
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        Node(
            package='image_to_gstreamer',
            executable='image_to_gstreamer_node',
            name='image_to_gstreamer_node',
            parameters=[
                {'input_topic': '/zed_node/left/image_rect_color'},
                {'host': LaunchConfiguration('host')},
                {'port': LaunchConfiguration('port')},
            ],
        )
    ])