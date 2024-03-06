
/usr/bin/cmake /home/hlf/Desktop/radar24_ws/src/get_depth -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/hlf/Desktop/radar24_ws/install/get_depth
/usr/bin/cmake --build /home/hlf/Desktop/radar24_ws/build/get_depth -- -j8 -l8
/usr/bin/cmake --install /home/hlf/Desktop/radar24_ws/build/get_depth
