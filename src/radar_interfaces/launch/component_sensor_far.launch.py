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
                    parameters=[
                      {
                        "camera_name": 'camera_far',
                        "camera_height": 1200,
                        "camera_width": 1920,
                        "camera_exp":2500.0,
                        "camera_gain": 23.98,
                        "camera_auto_exp": 0,
                        "camera_auto_maxexp": 4500.0,
                        "camera_auto_minexp": 100.0,
                        "camera_auto_gain": 0,
                        "camera_auto_maxgain": 17.0,
                        "camera_auto_mingain": 0.0,
                        "camera_auto_whitebalance": 0,
                      }
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='yolov5_detect',
                    plugin='yolov5_detect::Yolov5Detector',
                    name='far_yolov5_detect',
                    namespace='sensor_far',
                    parameters=[
                      {
                        "show_by_cv_or_msg": 0,
                        "camera_name": "sensor_far"
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
                        "camera_matrix": [2740.785753, 0.0, 942.857551, 0.0, 2730.148347, 595.336, 0.0, 0.0, 1.0],
                        "distortion_coefficient": [-0.118461, 0.884219, -0.000027, -0.001337, 0.0],
                        "uni_matrix": [0.158509, -0.987162, 0.0196718, 0.13549, 0.122233, -0.000151194, -0.992501, -0.0109141, 0.979762, 0.159725, 0.12064, -0.100546],
                        "length_of_cloud_queue": 10,
                        "image_width": 1920, # 1280
                        "image_height": 1200, # 1024,
                        "camera_name": "sensor_far",
                        "show_by_cv_or_msg": 0
                      },
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

