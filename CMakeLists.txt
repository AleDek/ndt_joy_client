cmake_minimum_required(VERSION 3.0.2)
project(ndt_joy_client)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  ndt_core_interface
  message_filters

)
include_directories( /usr/include/eigen3 )
find_package(iir REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landing_procedure
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable( ndt_joy_client src/ndt_joy_client.cpp ) #src/raise.cpp)
target_link_libraries( ndt_joy_client ${catkin_LIBRARIES} ${PCL_LIBRARIES} iir::iir)