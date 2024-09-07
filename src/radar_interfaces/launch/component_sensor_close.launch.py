import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    container = ComposableNodeContainer(
            namespace='sensor_close',
            name='sensor_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    namespace='sensor_close',
                    name='bayer_camera_driver',
                    package='bayer_camera_driver',
                    plugin='bayer_camera_driver::HikvisionDriver',
                    parameters=[
                      {
                        "camera_name": 'camera_close',
                        "camera_height": 1200,
                        "camera_width": 1920,
                        "camera_exp": 4000.0,
                        "camera_gain": 23.8,
                        "camera_auto_exp": 0,
                        "camera_auto_maxexp": 6500.0,
                        "camera_auto_minexp": 2000.0,
                        "camera_auto_gain": 0,
                        "camera_auto_maxgain": 24.0,
                        "camera_auto_mingain": 0.0,
                        "camera_auto_whitebalance": 0
                      }
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='yolov5_detect',
                    plugin='yolov5_detect::Yolov5Detector',
                    name='close_yolov5_detect',
                    namespace='sensor_close',
                    parameters=[
                      {
                        "show_by_cv_or_msg": 0,
                        "camera_name": "camera_close",
                        "rgb_or_bayer": True,
                        "light_gain": 2.0,
                        "saturation_gain": 1.6,
                        "image_width": 1920,
                        "image_height": 1200,
                        "roi_x": 1700,
                        "roi_y": 420,
                        "last_diff_time_threshold": 400.0,
                        "param_1": 250,
                        "param_2": 150
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='livox_ros2_driver',
                    plugin='livox_ros::LivoxDriver',
                    name='close_livox_ros2_driver',
                    namespace='sensor_close',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='get_depth',
                    plugin='get_depth::DepthSensor',
                    name='close_get_depth',
                    namespace='sensor_close',
                    parameters=[
                      {
                        "camera_matrix": [1406.31248, 0.0, 966.52234, 0.0, 1402.27487, 599.34296, 0.0, 0.0, 1.0],
                        "distortion_coefficient": [-0.068506, 0.128114, -0.000808, 0.001835, 0.0],
                        "uni_matrix": [-0.10095, -0.994887, -0.00309011, 0.136231, -0.0384396, 0.00700415, -0.999236, -0.0196554, 0.994149, -0.100754, -0.03895, 0.0116682],
                        "length_of_cloud_queue": 10,
                        "image_width": 1920,
                        "image_height": 1200,
                        "camera_name": "camera_close",
                        "show_by_cv_or_msg": 0
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

