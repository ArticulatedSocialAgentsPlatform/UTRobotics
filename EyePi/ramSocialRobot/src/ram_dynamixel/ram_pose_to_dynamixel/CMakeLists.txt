cmake_minimum_required(VERSION 2.8.3)
project(ram_pose_to_dynamixel)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(
        ram_pose_to_dynamixel_node src/ram_pose_to_dynamixel_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ram_pose_to_dynamixel_node
        ${catkin_LIBRARIES}
)

install(
        TARGETS ram_pose_to_dynamixel_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
