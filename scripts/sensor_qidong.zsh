source ~/home/hlf/Desktop/radar24_ws/install/setup.zsh
gnome-terminal -x bash -c "ros2 launch radar_interfaces component_sensor_close.launch.py"
sleep 1
gnome-terminal -x bash -c "ros2 launch radar_interfaces component_sensor_far.launch.py"
sleep 1
gnome-terminal -x bash -c "ros2 launch radar_interfaces pnp_solver.launch.py"
sleep 1
gnome-terminal -x bash -c "ros2 launch radar_interfaces small_map.launch.py"
