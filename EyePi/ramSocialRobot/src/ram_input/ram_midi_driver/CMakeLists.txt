cmake_minimum_required(VERSION 2.8.3)
project(ram_midi_driver)
find_package(
        catkin REQUIRED COMPONENTS
        rospy
        sensor_msgs
        ram_input_msgs
        ram_animation_msgs
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

install(
        PROGRAMS scripts/ram_midi_driver_input_node.py scripts/ram_midi_driver_output_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
