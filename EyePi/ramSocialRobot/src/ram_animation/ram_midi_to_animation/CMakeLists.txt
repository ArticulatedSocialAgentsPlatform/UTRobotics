cmake_minimum_required(VERSION 2.8.3)
project(ram_midi_to_animation)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        ram_input_msgs
        ram_animation_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ram_midi_to_animation
        #  CATKIN_DEPENDS message_generation message_runtime ram_midi_driver ram_animator roscpp
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        ram_midi_to_animation_node src/ram_midi_to_animation_node.cpp
)

target_link_libraries(
        ram_midi_to_animation_node
        ${catkin_LIBRARIES}
)

install(
        TARGETS ram_midi_to_animation_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
