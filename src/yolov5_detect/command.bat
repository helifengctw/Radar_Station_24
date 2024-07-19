
/usr/bin/cmake /home/hlf/Desktop/radar24_ws/src/yolov5_detect -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/hlf/Desktop/radar24_ws/install/yolov5_detect
/usr/bin/cmake --build /home/hlf/Desktop/radar24_ws/build/yolov5_detect -- -j8 -l8
/usr/bin/cmake --install /home/hlf/Desktop/radar24_ws/build/yolov5_detect
