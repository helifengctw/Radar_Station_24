cmake_minimum_required(VERSION 3.5)
project(get_depth)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(radar_interfaces REQUIRED)
find_package(tf2 REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#include_directories(${tf2_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/include/get_depth/)

### get_depth excutable
add_executable(get_depth src/get_depth_node.cpp)
target_include_directories(get_depth PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(get_depth rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)
target_link_libraries(get_depth ${OpenCV_LIBS})
#target_link_libraries(get_depth ${tf_LIBS})
target_link_libraries(get_depth ${PCL_LIBRARIES})


### get_depth component
ament_auto_add_library(depth_sensor SHARED src/get_depth_component.cpp)
target_include_directories(depth_sensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(depth_sensor ${PCL_LIBRARIES})
target_link_libraries(depth_sensor ${OpenCV_LIBS})
ament_target_dependencies(depth_sensor rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)

rclcpp_components_register_node(depth_sensor
        PLUGIN "get_depth::DepthSensor"
        EXECUTABLE depth_sensor_node
)

install(TARGETS get_depth
        DESTINATION lib/${PROJECT_NAME})

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
