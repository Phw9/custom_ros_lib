cmake_minimum_required(VERSION 3.0.2)
project(custom_ros_lib)

## Compile as C++17, necessary for PCL on Noetic
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

catkin_package()  # devel/lib에 실행파일 생성 (패키지화)

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

# livox2pcd
file(GLOB LIVOX2PCD_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/livox2pcd/*.cc")
add_executable(livox2pcd ${LIVOX2PCD_SOURCES})
target_include_directories(livox2pcd PRIVATE include/livox2pcd ${catkin_INCLUDE_DIRS})
target_link_libraries(livox2pcd PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ouster2pcd
file(GLOB OUSTER2PCD_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/ouster2pcd/*.cc")
add_executable(ouster2pcd ${OUSTER2PCD_SOURCES})
target_include_directories(ouster2pcd PRIVATE include/ouster2pcd ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster2pcd PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES})