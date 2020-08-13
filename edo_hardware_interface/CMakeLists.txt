cmake_minimum_required(VERSION 2.8.3)
project(edo_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    edo_core_msgs
    hardware_interface
    controller_manager
    control_msgs
    trajectory_msgs
    actionlib
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(roscpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES edo_hardware_interface
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS roscpp
)

## Build talker and listener
include_directories(
    include/
    include ${catkin_INCLUDE_DIRS}
)

add_library(edo_driver src/edo_driver.cpp)
target_link_libraries(edo_driver
	${catkin_LIBRARIES})

add_executable(hardware_interface_node src/edo_hardware_interface_node.cpp)
target_link_libraries(hardware_interface_node 
    edo_driver
    ${catkin_LIBRARIES})
