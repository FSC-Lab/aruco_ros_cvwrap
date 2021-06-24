cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros_cvwrap)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)


add_message_files(
  FILES
  ArucoTag.msg
  ArucoTagDetections.msg
)
  
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
    
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${BINARY_DIR}/devel/include/aruco_ros_cvwrap/
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(
  utils
  SHARED
  src/utils.cpp
)

add_executable(basic_detection src/basic_detection.cpp src/utils.cpp)
add_dependencies(basic_detection aruco_ros_cvwrap_generate_messages_cpp utils)
target_link_libraries(basic_detection
  ${utils}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${utils}
)