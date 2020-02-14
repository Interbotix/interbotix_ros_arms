cmake_minimum_required(VERSION 2.8.3)
project(interbotix_moveit_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interbotix_moveit
  message_generation
  moveit_commander
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  std_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveItPlan.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs interbotix_moveit message_runtime moveit_commander moveit_ros_planning_interface moveit_visual_tools roscpp rospy std_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(moveit_interface_node src/moveit_interface_node.cpp src/moveit_interface.cpp)
target_link_libraries(moveit_interface_node ${catkin_LIBRARIES})
add_dependencies(moveit_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/moveit_interface_gui
  scripts/moveit_python_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
