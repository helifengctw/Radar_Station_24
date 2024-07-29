import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 两个组件放在一个 container 里，也就是共享一个进程，可以降低负载
    # 通常情况下，开发阶段使用分开进程的方式，生产环境使用这种方式
    camera_params = os.path.join(
            get_package_share_directory('radar_interfaces'),
            'config',
            'camera_driver_params.yaml'
        )
    container = ComposableNodeContainer(
            namespace='sensor_close',
            name='sensor_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     namespace='sensor_close',
                #     name='bayer_camera_driver',
                #     package='bayer_camera_driver',
                #     plugin='bayer_camera_driver::HikvisionDriver',
                #     parameters=[
                #       {
                #         "camera_name": 'camera_close',
                #         "camera_height": 1200,
                #         "camera_width": 1920,
                #         "camera_exp": 3000.0,
                #         "camera_gain": 23.8,
                #         "camera_auto_exp": 0,
                #         "camera_auto_maxexp": 4500.0,
                #         "camera_auto_minexp": 100.0,
                #         "camera_auto_gain": 0,
                #         "camera_auto_maxgain": 17.0,
                #         "camera_auto_mingain": 0.0,
                #         "camera_auto_whitebalance": 1,
                #       }
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='yolov5_detect',
                    plugin='yolov5_detect::Yolov5Detector',
                    name='close_yolov5_detect',
                    namespace='sensor_close',
                    parameters=[
                      {
                        "show_by_cv_or_msg": 0,
                        "camera_name": "sensor_close",
                        "rgb_or_bayer": True,
                        "light_gain": 2.0,
                        "saturation_gain": 1.6,
                        "image_width": 1920, # 1280
                        "image_height": 1200, # 1024,
                        "roi_x": 1697,
                        "roi_y": 197
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                # ComposableNode(
                #     package='livox_ros2_driver',
                #     plugin='livox_ros::LivoxDriver',
                #     name='close_livox_ros2_driver',
                #     namespace='sensor_close',
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='get_depth',
                    plugin='get_depth::DepthSensor',
                    name='close_get_depth',
                    namespace='sensor_close',
                    parameters=[
                      {
                        "camera_matrix": [1406.312482, 0.000000, 966.522342, 0.000000, 1402.274874, 599.342961, 0.0, 0.0, 1.0],
                        "distortion_coefficient": [-0.068506, 0.128114, -0.000808, 0.001835, 0.0],
                        "uni_matrix": [-0.10095, -0.994887, -0.00309011, 0.136231, -0.0384396, 0.00700415, -0.999236, -0.0196554, 0.994149, -0.100754, -0.03895, 0.0116682],
                        "length_of_cloud_queue": 10,
                        "image_width": 1920, # 1280
                        "image_height": 1200, # 1024,
                        "camera_name": "sensor_close",
                        "show_by_cv_or_msg": 0
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

