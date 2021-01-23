cmake_minimum_required(VERSION 2.8.3)
project(weed_robot_navigation)

## Package
find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  geometry_msgs
  roscpp
  tf
  gazebo REQUIRED
  std_msgs REQUIRED
)

catkin_package(
  DEPENDS roscpp gazebo_ros
)

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(point_cloud_obstacle src/point_cloud_obstacle.cpp)
target_link_libraries(point_cloud_obstacle ${catkin_LIBRARIES})

