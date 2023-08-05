cmake_minimum_required(VERSION 3.0.2)
project(livox2las)

## Compile as C++17, necessary for PCL on Noetic
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf_conversions
  visualization_msgs
  cv_bridge
  image_transport  
  rospy
  roscpp
  livox_ros_driver2
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE) # 3.3 includes CUDA support
find_package(Open3D)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED PATHS)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem) # compatibility between CUDA 14 and C++17

# livox2las
file(GLOB LIVOX2LAS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/livox2las/*.cc")
add_executable(livox2las ${LIVOX2LAS_SOURCES})
target_include_directories(livox2las PRIVATE include/livox2las ${catkin_INCLUDE_DIRS})
target_link_libraries(livox2las PUBLIC ${catkin_LIBRARIES})