cmake_minimum_required(VERSION 2.8.3)
project(joints_sim)

find_package(
        catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(state_publisher src/state_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_publisher ${catkin_LIBRARIES})
