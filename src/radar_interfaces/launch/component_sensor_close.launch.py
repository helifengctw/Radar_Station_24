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
                        "camera_exp": 3500.0,
                        "camera_gain": 22.05,
                        "camera_auto_exp": 0,
                        "camera_auto_maxexp": 4500.0,
                        "camera_auto_minexp": 100.0,
                        "camera_auto_gain": 0,
                        "camera_auto_maxgain": 17.0,
                        "camera_auto_mingain": 0.0,
                        "camera_auto_whitebalance": 1,
                      }
                    ],
                    # 尽管多个组件合并在同一个进程，ros2 也是默认走 DDS 中间件通信
                    # 下面的参数，可以设置组件之间通过 intra-process 通信
                    # 理论上，intra-process 直接传递指针，效率更高
                    # 但是本样例太小了，实际测试无法看出使用 intra-process 的优势
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
                        "camera_name": "sensor_close"
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
                        "camera_matrix": [2099.033157, 0.0, 979.223578, 0.0, 2090.619881, 651.142891, 0.0, 0.0, 1.0],
                        "distortion_coefficient": [-0.074087, 0.357193, 0.000931, 0.001243, 0.0],
                        "uni_matrix": [-0.271642, -0.96196, -0.0290318, -0.154767, -0.0715673, 0.0502737, -0.996168, 0.0182244, 0.959734, -0.268524, -0.0825014, -0.00121669],
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

