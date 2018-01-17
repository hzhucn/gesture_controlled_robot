cmake_minimum_required(VERSION 2.8.3)
project(kteams_node)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  camera_info_manager
  tf
  kteams_driver)

find_package(PkgConfig REQUIRED)
pkg_check_modules(KTPROT ktprot REQUIRED)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs image_transport camera_info_manager tf kteams_driver
  )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${KTPROT_INCLUDE_DIRS}
  )

add_subdirectory(src)
