from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    use_nvidia = LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'

    container = ComposableNodeContainer(
        name='gstreamer_from_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_from_ros_node',
                parameters=[{
                    'input_topic': '/zed_node/left/image_rect_color',
                    'destination_ip': '10.0.0.154',
                    'destination_port': 5001,
                    'expected_input_fps': 15,
                    'bitrate': 500000,
                    'preset_level': 1,
                    'iframe_interval': 15,
                    'control_rate': 1,
                    'pt': 96,
                    'config_interval': 1,
                    'input_format': 'RGB',
                    'hw_encoder': use_nvidia,
                }],
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gst_nvidia_encoder',
            default_value='true',
            description='Use NVIDIA hardware H.265 encoder (nvv4l2h265enc). '
                        'Set false to use software x265enc.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
