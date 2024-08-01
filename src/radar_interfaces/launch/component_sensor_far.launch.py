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
                # ComposableNode(
                #     namespace='sensor_far',
                #     name='bayer_camera_driver',
                #     package='bayer_camera_driver',
                #     plugin='bayer_camera_driver::HikvisionDriver',
                #     parameters=[
                #       {
                #         "camera_name": 'camera_far',
                #         "camera_height": 1200,
                #         "camera_width": 1920,
                #         "camera_exp":2500.0,
                #         "camera_gain": 23.98,
                #         "camera_auto_exp": 0,
                #         "camera_auto_maxexp": 4500.0,
                #         "camera_auto_minexp": 100.0,
                #         "camera_auto_gain": 0,
                #         "camera_auto_maxgain": 24.0,
                #         "camera_auto_mingain": 0.0,
                #         "camera_auto_whitebalance": 0
                #       }
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='yolov5_detect',
                    plugin='yolov5_detect::Yolov5Detector',
                    name='far_yolov5_detect',
                    namespace='sensor_far',
                    parameters=[
                      {
                        "show_by_cv_or_msg": 0,
                        "camera_name": "camera_far",
                        "rgb_or_bayer": True,
                        "light_gain": 2.0,
                        "saturation_gain": 1.6,
                        "image_width": 1920,
                        "image_height": 1200,
                        "roi_x": 0, # 325,
                        "roi_y": 1200, # 975,
                        "last_diff_time_threshold": 600.0
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                # ComposableNode(
                    # package='livox_ros2_driver',
                    # plugin='livox_ros::LivoxDriver',
                    # name='close_livox_ros2_driver',
                    # namespace='sensor_far',
                    # extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='get_depth',
                    plugin='get_depth::DepthSensor',
                    name='far_get_depth',
                    namespace='sensor_far',
                    remappings=[('/sensor_far/livox/lidar', '/sensor_close/livox/lidar')],
                    parameters=[
                      {
                        "camera_matrix": [2786.15556, 0.0, 973.55417, 0.0, 2776.847274, 618.336654, 0.0, 0.0, 1.0],
                        "distortion_coefficient": [-0.102390, 0.924797, -0.002373, 0.002358, 0.0],
                        "uni_matrix": [-0.000301777, -0.999289, -0.0377013, -0.142598, 0.21708, 0.0367368, -0.975462, 0.0091706, 0.976154, -0.00847858, 0.216915, 0.0844117],
                        "length_of_cloud_queue": 10,
                        "image_width": 1920, # 1280
                        "image_height": 1200, # 1024,
                        "camera_name": "camera_far",
                        "show_by_cv_or_msg": 0
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

