cmake_minimum_required(VERSION 3.5)
project(serial_port)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(radar_interfaces REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/serial_port/)

#add_executable(serial_port src/serial_port_node.cpp
#        include/serial_port/serial_port.h)
#target_include_directories(serial_port PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(serial_port rclcpp std_msgs serial radar_interfaces)
#target_link_libraries(serial_port ${OpenCV_LIBS})

add_executable(serial_port_new src/serial_port_new_node.cpp)
target_include_directories(serial_port_new PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(serial_port_new rclcpp std_msgs serial radar_interfaces)
target_link_libraries(serial_port_new ${OpenCV_LIBS})

#install(TARGETS serial_port
#        DESTINATION lib/${PROJECT_NAME})
install(TARGETS serial_port_new
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

ament_package()
