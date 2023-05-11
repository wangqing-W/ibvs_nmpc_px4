#     rpg_quadrotor_mpc
#     A model predictive control implementation for quadrotors.
#     Copyright (C) 2017-2018 Philipp Foehn, 
#     Robotics and Perception Group, University of Zurich
#  
#     Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
#     https://github.com/uzh-rpg/rpg_quadrotor_control
# 
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 3.0.2)
project(mpc_pkg)

find_package(catkin_simple REQUIRED
std_msgs
sensor_msgs
nav_msgs
message_generation
# ibvs_pkg
# rpg_quadrotor_common
custom_msgs
)

catkin_simple()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ibvs_pkg
 CATKIN_DEPENDS geometry_msgs mav_msgs roscpp sensor_msgs std_msgs trajectory_msgs nav_msgs
#  DEPENDS system_lib
)

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")
