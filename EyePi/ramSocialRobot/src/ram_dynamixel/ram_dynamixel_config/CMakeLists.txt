cmake_minimum_required(VERSION 2.8.3)
project(ram_dynamixel_config)

find_package(
        catkin REQUIRED COMPONENTS
        dynamixel_controllers
        roscpp
        rospy
        std_msgs
)

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

install(
        FILES
        controller.launch
        shutdown_controller.launch
        manager.launch
        shutdown_manager.launch
        config.yml
        shutdown_config.yml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
