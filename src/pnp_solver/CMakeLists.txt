cmake_minimum_required(VERSION 3.5)
project(pnp_solver)

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
find_package(radar_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(ndt_omp_ros2 REQUIRED)
find_package(OpenMP)

file(GLOB PCL_LIBRARIES /usr/lib/libpcl_*)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.14/pcl)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${lifecycle_msgs_INCLUDE_DIRS}
        ${rclcpp_lifecycle_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
)

add_executable(pnp_solver src/pnp_solver.cpp)
target_include_directories(pnp_solver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pnp_solver rclcpp std_msgs radar_interfaces)
target_link_libraries(pnp_solver ${OpenCV_LIBS})

add_executable(icp_solver src/icp_solver.cpp)
target_include_directories(icp_solver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(icp_solver
        rclcpp
        std_msgs
        radar_interfaces
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
        tf2_eigen
        geometry_msgs
        sensor_msgs
        nav_msgs
        pcl_conversions
#        ndt_omp_ros2
)
target_link_libraries(icp_solver ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS pnp_solver
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS icp_solver
        DESTINATION lib/${PROJECT_NAME})

#install(
#        DIRECTORY launch
#        DESTINATION share/${PROJECT_NAME}/)
#
#install(
#        DIRECTORY config
#        DESTINATION share/${PROJECT_NAME}/)

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
