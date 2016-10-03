cmake_minimum_required(VERSION 2.8.3)
project(ram_motion_detection)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        ram_input_msgs
        dynamic_reconfigure
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
        cfg/RamMotionDetection.cfg
)

catkin_package(
)

## Declare a C++ executable
add_executable(
        ram_motion_detection_node src/ram_motion_detection_node.cpp
)

add_dependencies(
        ram_motion_detection_node ${PROJECT_NAME}_gencfg
)

# Add debug flags
#add_definitions(-g)

find_package(OpenCV REQUIRED)
IF (NOT OpenCV_FOUND)
    MESSAGE(FATAL_ERROR "OpenCV not found")
ENDIF ()
find_package(raspicam REQUIRED
        PATHS /usr/local/lib
        )
IF (NOT raspicam_CV_FOUND)
    MESSAGE(FATAL_ERROR "Raspicam not found")
ENDIF ()
include_directories(
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
        ram_motion_detection_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${raspicam_CV_LIBS}
)

## Mark executables and/or libraries for installation
install(
        TARGETS ram_motion_detection_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()