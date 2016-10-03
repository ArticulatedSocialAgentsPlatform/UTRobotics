cmake_minimum_required(VERSION 2.8.3)
project(ram_animator)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        ram_animation_msgs
        ram_output_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include)

## Declare a C++ executable
add_executable(
        ram_animator_node src/ram_animator_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ram_animator_node
        ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(
        TARGETS ram_animator_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)