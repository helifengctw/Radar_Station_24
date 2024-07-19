cmake_minimum_required(VERSION 3.8)
project(bayer_camera_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/bayer_camera_driver/include
        /opt/MVS/lib/64/
        /usr/lib/x86_64-linux-gnu/
)

# find ROS2 dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(radar_interfaces REQUIRED)
ament_auto_find_build_dependencies()

# find hikvision sdk
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(MVS REQUIRED)

#find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE DRIVER_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hikvision_ros2_driver.cpp)
ament_auto_add_library(hikvision_driver SHARED ${DRIVER_LIB_SRCS})
target_include_directories(hikvision_driver PUBLIC include
  /opt/MVS/include)
target_include_directories(hikvision_driver PRIVATE ${MVS_INCLUDE_DIR})
target_link_libraries(hikvision_driver ${MVS_LIBRARIES})
ament_target_dependencies(hikvision_driver rclcpp std_msgs sensor_msgs image_transport radar_interfaces)


rclcpp_components_register_node(hikvision_driver
        PLUGIN "bayer_camera_driver::HikvisionDriver"
        EXECUTABLE hikvision_driver_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()


