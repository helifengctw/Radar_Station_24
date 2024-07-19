from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    radar_params = os.path.join(
        get_package_share_directory('radar_interfaces'),
        'config',
        'radar_params.yaml'
    	)
    calib_default_points = os.path.join(
        get_package_share_directory('radar_interfaces'),
        'config',
        'calib_default_point_race.yaml'
    )
    small_map_params = os.path.join(
        get_package_share_directory('radar_interfaces'),
        'config',
        'small_map_params.yaml'
    )

    Node_pnp_solver = Node(
        package="pnp_solver",
        executable="pnp_solver",
        name="pnp_solver",
        output="screen",
        emulate_tty=True,
        parameters=[radar_params, calib_default_points],
        respawn=True
    )

    Node_yolov5_detect = Node(
        package="yolov5_detect",
        executable="yolov5_detect_test",
        name="yolov5_detect_test",
        output="screen",
        emulate_tty=True,
        respawn=True
    )

    Node_get_depth = Node(
        package="get_depth",
        executable="get_depth",
        name="get_depth",
        output="screen",
        emulate_tty=True,
        respawn=True
    )

    Node_small_map = Node(
        package="small_map",
        executable="small_map",
        name="small_map",
        output="screen",
        emulate_tty=True,
        respawn=True,
        parameters=[small_map_params]
    )

    Node_serial_port = Node(
        package="serial_port",
        executable="serial_port_test",
        name="serial_port_test",
        output="screen",
        emulate_tty=True,
        respawn=True
    )

    return LaunchDescription([
        Node_pnp_solver, Node_small_map
    ])
