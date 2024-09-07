import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    container = ComposableNodeContainer(
            namespace='sensor_far',
            name='sensor_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    namespace='sensor_far',
                    name='bayer_camera_driver',
                    package='bayer_camera_driver',
                    plugin='bayer_camera_driver::HikvisionDriver',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='yolov5_detect',
                    plugin='yolov5_detect::Yolov5Detector',
                    name='far_yolov5_detect',
                    namespace='sensor_far',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='get_depth',
                    plugin='get_depth::DepthSensor',
                    name='far_get_depth',
                    namespace='sensor_far',
                    remappings=[('/sensor_far/livox/lidar', '/sensor_close/livox/lidar')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

