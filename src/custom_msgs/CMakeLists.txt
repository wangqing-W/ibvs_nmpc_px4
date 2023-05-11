cmake_minimum_required(VERSION 3.10.0)
project(custom_msgs)

set(CMAKE_CXX_STANDARD 11)

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs

  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
 
)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   nav_msgs
# )

# add_message_files(
#   DIRECTORY msg
#   FILES
#   FlatTarget.msg
#   ImgRecog.msg
#   PurposeWp.msg
#   depthRecog.msg
#   ParkingRecog.msg
#   CircleRecog.msg
#   ControlCommand.msg
# )


# catkin_package(
#     CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime std_msgs 
# )
cs_install()
cs_export()

