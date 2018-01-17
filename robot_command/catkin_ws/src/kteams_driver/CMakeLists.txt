cmake_minimum_required(VERSION 2.8.3)
project(kteams_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(KTPROT ktprot REQUIRED)
set(CMAKE_LDFLAGS "-ljpeg")
list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kteams_driver
  DEPENDS ktprot
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${KTPROT_INCLUDE_DIRS}
)
link_directories(${KTPROT_LINK_DIRECTORIES})

add_subdirectory(src)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}
)
