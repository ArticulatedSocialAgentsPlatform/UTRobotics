cmake_minimum_required(VERSION 2.8.3)
project(ram_safe_shutdown)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        dynamixel_msgs
        ram_animation_msgs
        ram_input_msgs
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        ram_safe_shutdown_node src/ram_safe_shutdown_node.cpp
)

target_link_libraries(
        ram_safe_shutdown_node
        ${catkin_LIBRARIES}
)

install(
        TARGETS ram_safe_shutdown_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
