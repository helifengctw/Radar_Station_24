cmake_minimum_required(VERSION 3.10)
project(yolov5_detect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wpedantic -Wextra -Wall) #
endif()

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)


### Notes: "find_package(*** REQUIRED)" is to find the dependencies of ROS2 SYSTEM;

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
#　set(radar_interfaces_DIR ~/Desktop/radar24_ws/install/radar_interfaces/share/radar_interfaces/cmake)
find_package(radar_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()


### "include_directories(${***_INCLUDE_DIRS})" is required additionally, 
### while finding dependecies out of ROS2 SYSTEM

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
message('OPENCV FOUND: ${OpenCV_FOUND}')

### Notes: prepration for the construction of library 
### and executable files

include_directories(${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/deepsort/include/
	${PROJECT_SOURCE_DIR}/include/yolov5_detect/
    ${EIGEN3_INCLUDE_DIRS}
)
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/calibrator.cpp
	${PROJECT_SOURCE_DIR}/src/model.cpp
	${PROJECT_SOURCE_DIR}/src/postprocess.cpp
	${PROJECT_SOURCE_DIR}/src/preprocess.cu
	${PROJECT_SOURCE_DIR}/src/yolov5_det.cpp
)
file(GLOB_RECURSE TEST_SRCS ${PROJECT_SOURCE_DIR}/src/calibrator.cpp
        ${PROJECT_SOURCE_DIR}/src/model.cpp
        ${PROJECT_SOURCE_DIR}/src/postprocess.cpp
        ${PROJECT_SOURCE_DIR}/src/preprocess.cu
        ${PROJECT_SOURCE_DIR}/src/yolov5_det_test.cpp
)
file(GLOB_RECURSE COMPONENT_SRCS ${PROJECT_SOURCE_DIR}/src/calibrator.cpp
        ${PROJECT_SOURCE_DIR}/src/model.cpp
        ${PROJECT_SOURCE_DIR}/src/postprocess.cpp
        ${PROJECT_SOURCE_DIR}/src/preprocess.cu
        ${PROJECT_SOURCE_DIR}/src/yolov5_det_component.cpp
)
file(GLOB_RECURSE PLUGIN_SRCS ${PROJECT_SOURCE_DIR}/src/yololayer.cu)
aux_source_directory(${PROJECT_SOURCE_DIR}/deepsort/src DEEPSORT_SRC_DIR)


### ===== deepsort =====
add_library(deepsort SHARED ${DEEPSORT_SRC_DIR})
target_link_libraries(deepsort
        ${CUDA_LIBS} ${OpenCV_LIBS}
        cudart nvinfer nvonnxparser
)

### Notes: two worlds fouction as construct library and adding dependencies
add_library(myplugins SHARED ${PLUGIN_SRCS})
target_link_libraries(myplugins nvinfer cudart)

### Hello_world executable
add_executable(Hello_world src/Hello_world.cpp)
target_include_directories(Hello_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(Hello_world ${OpenCV_LIBS})

### yolov5_detect executable
add_executable(yolov5_detect ${SRCS})
target_include_directories(yolov5_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolov5_detect rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)
target_link_libraries(yolov5_detect nvinfer)
target_link_libraries(yolov5_detect cudart)
target_link_libraries(yolov5_detect deepsort)
target_link_libraries(yolov5_detect myplugins)
target_link_libraries(yolov5_detect ${OpenCV_LIBS})

### yolov5_detect_test executable
add_executable(yolov5_detect_test ${TEST_SRCS})
target_include_directories(yolov5_detect_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolov5_detect_test rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)
target_link_libraries(yolov5_detect_test nvinfer)
target_link_libraries(yolov5_detect_test deepsort)
target_link_libraries(yolov5_detect_test cudart)
target_link_libraries(yolov5_detect_test myplugins)
target_link_libraries(yolov5_detect_test ${OpenCV_LIBS})

### yolov5_detect_component
ament_auto_add_library(yolov5_detector SHARED ${COMPONENT_SRCS})
target_include_directories(yolov5_detector PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(yolov5_detector nvinfer)
target_link_libraries(yolov5_detector deepsort)
target_link_libraries(yolov5_detector cudart)
target_link_libraries(yolov5_detector myplugins)
target_link_libraries(yolov5_detector ${OpenCV_LIBS})
ament_target_dependencies(yolov5_detector rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)

rclcpp_components_register_node(yolov5_detector
        PLUGIN "yolov5_detect::Yolov5Detector"
        EXECUTABLE yolov5_detector_node
)

#add_executable(yolov5_detect_component ${COMPONENT_SRCS})
#target_include_directories(yolov5_detect_component PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(yolov5_detect_component rclcpp std_msgs sensor_msgs cv_bridge image_transport radar_interfaces)
#target_link_libraries(yolov5_detect_component nvinfer)
#target_link_libraries(yolov5_detect_component deepsort)
#target_link_libraries(yolov5_detect_component cudart)
#target_link_libraries(yolov5_detect_component myplugins)
#target_link_libraries(yolov5_detect_component ${OpenCV_LIBS})

set_target_properties(yolov5_detect PROPERTIES CUDA_ARCHITECTURES "35;50;72")
set_target_properties(yolov5_detect_test PROPERTIES CUDA_ARCHITECTURES "35;50;72")
set_target_properties(yolov5_detector PROPERTIES CUDA_ARCHITECTURES "35;50;72")
set_target_properties(myplugins PROPERTIES CUDA_ARCHITECTURES "35;50;72")


install(TARGETS Hello_world
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS yolov5_detect
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS yolov5_detect_test
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