cmake_minimum_required(VERSION 2.8.3)
project(asap_ros_bridge)
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
        PROGRAMS ../../ram_asap/asap_ros_bridge/scripts/relay.py
        ../../ram_asap/asap_ros_bridge/scripts/xml_conversion.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
