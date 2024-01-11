cmake_minimum_required(VERSION 2.8.3)
project(LinK3D)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs pcl_ros roscpp roslib rospy sensor_msgs std_msgs visualization_msgs
    DEPENDS EIGEN3 PCL
)


include_directories(
    include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
src/LinK3D_extractor.cpp )

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES} 
${Boost_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(link3d_bin src/BinMain.cpp)
target_link_libraries(link3d_bin ${PROJECT_NAME})

add_executable(link3d_rosbag src/RosbagMain.cpp)
target_link_libraries(link3d_rosbag ${PROJECT_NAME})


