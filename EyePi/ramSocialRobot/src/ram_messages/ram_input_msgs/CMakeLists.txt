cmake_minimum_required(VERSION 2.8.3)
project(ram_input_msgs)

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
        Emotion.msg
        Midi.msg
        Saliency.msg
        Sequence.msg
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
