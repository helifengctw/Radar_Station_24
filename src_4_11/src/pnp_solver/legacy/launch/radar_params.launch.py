from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    radar_params = os.path.join(
        get_package_share_directory('pnp_solver'),
        'config',
        'radar_params.yaml'
    	)
    calib_default_points = os.path.join(
        get_package_share_directory('pnp_solver'),
        'config',
        'calib_default_point.yaml'
    )

    Node_pnp_solver = Node(
        package="pnp_solver",
        executable="pnp_solver",
        name="pnp_solver",
        output="screen",
        emulate_tty=True,
        parameters=[radar_params, calib_default_points]
    )

    return LaunchDescription([
        Node_pnp_solver
    ])