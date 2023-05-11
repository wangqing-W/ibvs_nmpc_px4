cmake_minimum_required(VERSION 3.0.2)
project(circle_detection)

find_package(catkin REQUIRED
    cv_bridge
    message_generation
    roscpp
    std_msgs
    custom_msgs
    image_transport
    rospy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge opencv roscpp std_msgs custom_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)

find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(include
  	${Eigen3_INCLUDE_DIRS} 
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node 
    src/CircleDetection.cpp
    src/circle_detection_node.cpp
    src/ED-LIB.cpp
    src/ED.cpp
    src/EDCircles.cpp
    src/EDColor.cpp
    src/EDLines.cpp
    src/EDPF.cpp
    src/NFA.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

