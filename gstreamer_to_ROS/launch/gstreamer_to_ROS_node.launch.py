from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    use_nvidia = LaunchConfiguration('use_nvidia').perform(context).lower() == 'true'

    container = ComposableNodeContainer(
        name='gstreamer_to_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='gstreamer_to_ROS',
                plugin='gstreamer_to_ros::GStreamerToROS',
                name='gstreamer_to_ROS_node',
                parameters=[{
                    'host': '0.0.0.0',
                    'port': 5001,
                    'output_topic': '/camera/image_raw',
                    'hw_decoder': use_nvidia,
                }],
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_nvidia',
            default_value='true',
            description='Use NVIDIA hardware H.265 decoder (nvh265dec). '
                        'Set false to use software avdec_h265.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
