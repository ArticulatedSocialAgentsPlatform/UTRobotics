cmake_minimum_required(VERSION 2.8.3)
project(ram_hmmm)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        ram_animation_msgs
        ram_input_msgs
        ram_output_msgs
        ram_animator
        dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
IF (NOT OpenCV_FOUND)
    MESSAGE(FATAL_ERROR "OpenCV not found")
ENDIF ()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
        cfg/RamHmmm.cfg
)

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(
        ram_hmmm_node src/ram_hmmm_node.cpp
)

add_dependencies(
        ram_hmmm_node ${PROJECT_NAME}_gencfg
)

target_link_libraries(
        ram_hmmm_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

install(
        TARGETS ram_hmmm_node
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