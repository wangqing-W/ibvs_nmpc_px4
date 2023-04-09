/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/Imu.h>
#include "ibvs_pkg/xyzyawVel.h"
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/LowLevelFeedback.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "ibvs_mpc1/ibvs_mpc1_controller.h"
#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

/* Declaring namespaces */
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ibvs_mpc1");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // cout << "DB 000" << endl;
  ibvs_mpc1::IbvsMpc1Controller<float>
    ibvsMpc1Controller(nh, pnh);
  // // Declare ROS Publisher
  ros::Publisher arm_pub_;
  arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);
  // UAV state initailizatio
  ros::Rate rate(30);
  // double control_command_delay_ = 0.05;
  // Arm bridge
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);
  while(ros::ok())
  {
    // get a msg
    ros::spinOnce();
    rate.sleep();
  }
  // ros::spin();

  return 0;
}