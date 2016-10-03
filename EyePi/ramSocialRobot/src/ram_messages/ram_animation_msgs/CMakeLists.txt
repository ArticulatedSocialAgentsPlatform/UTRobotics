cmake_minimum_required(VERSION 2.8.3)
project(ram_animation_msgs)

find_package(
        catkin REQUIRED
        COMPONENTS
        std_msgs
        message_generation
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_message_files(
        FILES
        Animation.msg
        EnabledModule.msg
        Face.msg
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
        INCLUDE_DIRS include
)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)