cmake_minimum_required(VERSION 2.8.3)
project(robotathome_at_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin_packages dependent projects also need

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(generate_depth_cloud
  src/depth_cloud.cpp
)

add_executable(pub_odom
  src/pub_odom.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(generate_depth_cloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(pub_odom
  ${catkin_LIBRARIES}
)
