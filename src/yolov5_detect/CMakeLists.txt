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
  add_compile_options(-Wall -Wextra) #  -Wpedantic
endif()

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)
message("CUDA ARCHITECTURES IS : "${CMAKE_CUDA_COMPILER})

# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)


### Notes: "find_package(*** REQUIRED)" is to find the dependencies
### of ROS2 SYSTEM; 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


### "include_directories(${***_INCLUDE_DIRS})" is required additionally, 
### while finding dependecies out of ROS2 SYSTEM

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
message('OPENCV FOUND: ${OpenCV_FOUND}')

### Notes: prepration for the construction of library 
### and executable files

include_directories(${PROJECT_SOURCE_DIR}/src/
	${PROJECT_SOURCE_DIR}/include/yolov5_detect/)
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/calibrator.cpp
	${PROJECT_SOURCE_DIR}/src/model.cpp
	${PROJECT_SOURCE_DIR}/src/postprocess.cpp
	${PROJECT_SOURCE_DIR}/src/preprocess.cu
	${PROJECT_SOURCE_DIR}/src/yolov5_det.cpp)
file(GLOB_RECURSE PLUGIN_SRCS ${PROJECT_SOURCE_DIR}/src/yololayer.cu)


### Notes: two worlds fouction as construct library and adding dependencies
add_library(myplugins SHARED ${PLUGIN_SRCS})
target_link_libraries(myplugins nvinfer cudart)

add_executable(Hello_world src/Hello_world.cpp)
target_include_directories(Hello_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(yolov5_detect ${SRCS})
target_include_directories(yolov5_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolov5_detect rclcpp std_msgs)
target_link_libraries(yolov5_detect nvinfer)
target_link_libraries(yolov5_detect cudart)
target_link_libraries(yolov5_detect myplugins)
target_link_libraries(yolov5_detect ${OpenCV_LIBS})


set_target_properties(yolov5_detect PROPERTIES CUDA_ARCHITECTURES "35;50;72")
set_target_properties(myplugins PROPERTIES CUDA_ARCHITECTURES "35;50;72")


install(TARGETS Hello_world
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS yolov5_detect
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
