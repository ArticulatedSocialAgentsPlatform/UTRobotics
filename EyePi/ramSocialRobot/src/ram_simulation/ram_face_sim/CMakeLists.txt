cmake_minimum_required(VERSION 2.8.3)
project(ram_face_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        ram_animation_msgs
        geometry_msgs
        nav_msgs
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ram_face_sim_node src/ram_face_sim_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ram_face_sim_node
        ${catkin_LIBRARIES}
)

install(
        TARGETS ram_face_sim_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
