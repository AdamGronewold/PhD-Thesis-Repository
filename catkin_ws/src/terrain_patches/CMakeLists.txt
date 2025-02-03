cmake_minimum_required(VERSION 3.0.2)
project(terrain_patches)

## Compile as C++11, supported in ROS Kinetic and newer - Noetic ideal

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    rospy
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
