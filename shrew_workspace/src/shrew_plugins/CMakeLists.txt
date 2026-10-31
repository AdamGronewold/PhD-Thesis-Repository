#http: //wiki.ros.org/catkin/CMakeLists.txt
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(shrew_plugins)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  rospy
  std_msgs
  sensor_msgs
  
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    rospy
    std_msgs
    sensor_msgs
)

include_directories(
	include
	${Boost_INCLUDE_DIR} 
	${catkin_INCLUDE_DIRS} 
	${GAZEBO_INCLUDE_DIRS}
	)

link_directories(
	${GAZEBO_LIBRARY_DIRS}
	)

add_library(shrew_plugin SHARED shrew_plugin.cc)
add_executable(simple_wheel_vel simple_wheel_vel.cc)
add_executable(full_sim_world_tf2_broadcaster full_sim_world_tf2_broadcaster.cc)

target_link_libraries(simple_wheel_vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(shrew_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(full_sim_world_tf2_broadcaster ${catkin_LIBRARIES})
