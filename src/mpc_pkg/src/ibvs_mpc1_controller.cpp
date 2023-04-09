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


#include "ibvs_mpc1/ibvs_mpc1_controller.h"
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <ctime>
using namespace std;
namespace ibvs_mpc1 {

template<typename T>
IbvsMpc1Controller<T>::IbvsMpc1Controller(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& topic) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(IbvsMpc1Wrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero()) {
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(topic, 1);

  // sub_point_of_interest_ = nh_.subscribe("mpc/point_of_interest", 1,
  //                                        &MpcController<T>::pointOfInterestCallback, this);
  sub_autopilot_off_ = nh_.subscribe("autopilot/off", 1,
                                     &IbvsMpc1Controller<T>::offCallback, this);
  imu_sub = nh_.subscribe("/hummingbird/imu", 1, &IbvsMpc1Controller<T>::imuCallback, this);
  refVel_sub = nh_.subscribe("/hummingbird/reference_vel", 1, &IbvsMpc1Controller<T>::refVelCallback, this);
  odom_sub = nh_.subscribe("/hummingbird/ground_truth/odometry", 1, &IbvsMpc1Controller<T>::estOdomCallback, this);
  control_command_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>("/hummingbird/autopilot/control_command_input", 1);
  arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&IbvsMpc1Wrapper<T>::prepare, mpc_wrapper_);
}

// template<typename T>
// void MpcController<T>::pointOfInterestCallback(
//     const geometry_msgs::PointStamped::ConstPtr& msg) {
//   point_of_interest_(0) = msg->point.x;
//   point_of_interest_(1) = msg->point.y;
//   point_of_interest_(2) = msg->point.z;
//   mpc_wrapper_.setPointOfInterest(point_of_interest_);
// }

template<typename T>
void IbvsMpc1Controller<T>::offCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  solve_from_scratch_ = true;
}

template<typename T>
void IbvsMpc1Controller<T>::imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
      
	uav_odom.pose.pose.orientation.w = msg->orientation.w;
	uav_odom.pose.pose.orientation.x = msg->orientation.x;
	uav_odom.pose.pose.orientation.y = msg->orientation.y;
	uav_odom.pose.pose.orientation.z = msg->orientation.z;
  mav_quad.w() = msg->orientation.w;
	mav_quad.x() = msg->orientation.x;
	mav_quad.y() = msg->orientation.y;
	mav_quad.z() = msg->orientation.z;
	// cout << "uav_odom_Orientation, w, x, y, z:" << uav_odom.pose.pose.orientation.w << ", " << uav_odom.pose.pose.orientation.x
  //      << ", " << uav_odom.pose.pose.orientation.y << ", " << uav_odom.pose.pose.orientation.z << endl;
}

template<typename T>
void IbvsMpc1Controller<T>::estOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
  // uav_odom.twist.twist.linear.x = msg->twist.twist.linear.x;
  // uav_odom.twist.twist.linear.y = msg->twist.twist.linear.y;
  // uav_odom.twist.twist.linear.z = msg->twist.twist.linear.z;
  Eigen::Vector3d vel_b, vel_a;
  vel_b.x()=msg->twist.twist.linear.x;
  vel_b.y()=msg->twist.twist.linear.y;
  vel_b.z()=msg->twist.twist.linear.z;
  vel_a = mav_quad * vel_b;
  uav_odom.twist.twist.linear.x = vel_a.x();
  uav_odom.twist.twist.linear.y = vel_a.y();
  uav_odom.twist.twist.linear.z = vel_a.z();

  // cout << "uav_odom_Velocity, x, y, z:" << uav_odom.twist.twist.linear.x << ", " << uav_odom.twist.twist.linear.y
  //      << ", " << uav_odom.twist.twist.linear.z << endl;
}

template<typename T>
void IbvsMpc1Controller<T>::refVelCallback(const ibvs_pkg::xyzyawVel::ConstPtr &msg)
{
  ref_vel.x() = msg->Vx;
  ref_vel.y() = msg->Vy;
  ref_vel.z() = msg->Vz;
  ref_vyaw = msg->Vyaw;
  // cout << "ref_vel, x, y, z, yaw:" << ref_vel.x() << ", "
  //      << ref_vel.y() << ", " << ref_vel.z() << ", " << ref_vyaw << endl;
  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time =
    wall_time_now + ros::Duration(control_command_delay_);
  // cout << "Current State is:" << autopilot_helper_.getCurrentAutopilotState() << endl;
  // Run the IbvsMpc1Controller with the estimated state and reference state
  received_state_est_ = quadrotor_common::QuadStateEstimate(uav_odom);
  reference_state_ = quadrotor_common::TrajectoryPoint();
  reference_state_.velocity = ref_vel;
  reference_state_.bodyrates = Eigen::Vector3d(0.0, 0.0, ref_vyaw);
  reference_state_.heading =
        quadrotor_common::quaternionToEulerAnglesZYX(mav_quad).z();
  reference_trajectory_ = quadrotor_common::Trajectory(reference_state_);
  cout << "--------------------INPUT------------------" << endl;
  cout << "uav_odom_Orientation, w, x, y, z:" << uav_odom.pose.pose.orientation.w << ", " << uav_odom.pose.pose.orientation.x
      << ", " << uav_odom.pose.pose.orientation.y << ", " << uav_odom.pose.pose.orientation.z << endl;
  cout << "uav_odom_Velocity, x, y, z:" << uav_odom.twist.twist.linear.x << ", " << uav_odom.twist.twist.linear.y
      << ", " << uav_odom.twist.twist.linear.z << endl;
  cout << "ref_vel, x, y, z, yaw:" << ref_vel.x() << ", "
      << ref_vel.y() << ", " << ref_vel.z() << ", " << ref_vyaw << endl;
  cout << "--------------------INPUT------------------" << endl;
  cout << "--------------------OUTPUT------------------" << endl;
  quadrotor_common::ControlCommand control_cmd = this->run(
      received_state_est_, reference_trajectory_, this->params_);
  // control_cmd.timestamp = wall_time_now;
  // control_cmd.expected_execution_time = command_execution_time;

  // Publish the control command
  quadrotor_msgs::ControlCommand control_cmd_msg;
  control_cmd_msg = control_cmd.toRosMessage();
  cout << "[INFO] control command bodyrates:" << control_cmd_msg.bodyrates.x << ", "
        << control_cmd_msg.bodyrates.y << ", " << control_cmd_msg.bodyrates.z << endl;
  cout << "[INFO] control command trust:" << control_cmd_msg.collective_thrust << endl;
  cout << "--------------------OUTPUT------------------" << endl;
  // control_cmd_msg.collective_thrust = 10;
  // control_cmd_msg.bodyrates.x = 0;
  // control_cmd_msg.bodyrates.y = 1;
  // control_cmd_msg.bodyrates.z = -control_cmd_msg.bodyrates.z;
  control_command_pub_.publish(control_cmd_msg);
}

template<typename T>
quadrotor_common::ControlCommand IbvsMpc1Controller<T>::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

template<typename T>
quadrotor_common::ControlCommand IbvsMpc1Controller<T>::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::Trajectory& reference_trajectory,
    const IbvsMpc1Params<T>& params) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  preparation_thread_.join();

  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setVelTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Publish the predicted trajectory.
  // publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&IbvsMpc1Controller<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  return updateControlCommand(predicted_states_.col(0),
                              predicted_inputs_.col(0),
                              call_time);
}

template<typename T>
bool IbvsMpc1Controller<T>::setStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  // est_state_(kPosX) = state_estimate.position.x();
  // est_state_(kPosY) = state_estimate.position.y();
  // est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kPosX) = 0;
  est_state_(kPosY) = 0;
  est_state_(kPosZ) = 0;
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
  // cout << "[INFO] State estimaion:" << endl
  //      << "orientation " << est_state_(kOriW) << ", " << est_state_(kOriX) << ", " << est_state_(kOriY) << ", " << est_state_(kOriZ) << endl
  //      << "velocity " << est_state_(kVelX) << ", " << est_state_(kVelY) << ", " << est_state_(kVelZ) << endl;
  return quaternion_norm_ok;
}

template<typename T>
bool IbvsMpc1Controller<T>::setReference(
    const quadrotor_common::Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory.points.size() == 1) {
    // cout << "[INFO] No trajectory generated!!!" << endl;
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    // cout << "[INFO] heading is:" << reference_trajectory.points.front().heading << endl;
    q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().velocity.template cast<T>()
    ).finished().replicate(1, kSamples + 1);

    acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
        reference_trajectory.points.front().bodyrates.template cast<T>()
    ).finished().replicate(1, kSamples + 1);
    // cout << "[INFO] reference_states_:" << endl
    //      << "position " << reference_trajectory.points.front().position.x() << ", " << reference_trajectory.points.front().position.y() << ", " << reference_trajectory.points.front().position.z() << endl
    //      << "orientation " << q_orientation.w() << ", " << q_orientation.x() << ", " << q_orientation.y() << ", " << q_orientation.z() << endl
    //      << "velocity " << reference_trajectory.points.front().velocity.x() << ", " << reference_trajectory.points.front().velocity.y() << ", " << reference_trajectory.points.front().velocity.z() << endl;
    // cout << "[INFO] reference_inputs_:" << acceleration.norm() << ", "
    //      << reference_trajectory.points.front().bodyrates.x() << ", " << reference_trajectory.points.front().bodyrates.y() << ", " << reference_trajectory.points.front().bodyrates.z() << endl;
  } else { 
    // cout << "[INFO] There is a trajectory generated!!!" << endl;
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      q_orientation = q_heading * iterator->orientation.template cast<T>();
      reference_states_.col(i) << iterator->position.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          iterator->velocity.template cast<T>();
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      acceleration << iterator->acceleration.template cast<T>() - gravity;
      reference_inputs_.col(i) << acceleration.norm(),
          iterator->bodyrates.template cast<T>();
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
quadrotor_common::ControlCommand IbvsMpc1Controller<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

  quadrotor_common::ControlCommand command;

  command.timestamp = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);

  // cout << "[INFO] rpg_mpc control command:" << command.collective_thrust << ", " << command.bodyrates.x() << ", "
  //      << command.bodyrates.y() << ", " << command.bodyrates.z() << endl;
  return command;
}

// template<typename T>
// bool MpcController<T>::publishPrediction(
//     const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
//     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
//     ros::Time& time) {
//   nav_msgs::Path path_msg;
//   path_msg.header.stamp = time;
//   path_msg.header.frame_id = "world";
//   geometry_msgs::PoseStamped pose;
//   T dt = mpc_wrapper_.getTimestep();

//   for (int i = 0; i < kSamples; i++) {
//     pose.header.stamp = time + ros::Duration(i * dt);
//     pose.header.seq = i;
//     // pose.pose.position.x = states(kPosX, i);
//     // pose.pose.position.y = states(kPosY, i);
//     // pose.pose.position.z = states(kPosZ, i);
//     // pose.pose.orientation.w = states(kOriW, i);
//     // pose.pose.orientation.x = states(kOriX, i);
//     // pose.pose.orientation.y = states(kOriY, i);
//     // pose.pose.orientation.z = states(kOriZ, i);
//     pose.pose.velocity.x = states(kVelX, i);
//     pose.pose.velocity.y = states(kVelY, i);
//     pose.pose.velocity.z = states(kVelZ, i);
//     path_msg.poses.push_back(pose);
//   }

//   pub_predicted_trajectory_.publish(path_msg);

//   return true;
// }

template<typename T>
void IbvsMpc1Controller<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool IbvsMpc1Controller<T>::setNewParams(IbvsMpc1Params<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_, params.max_quad_xyz_, 
      params.max_bodyrate_xy_, params.max_bodyrate_z_);
  // mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class IbvsMpc1Controller<float>;

template
class IbvsMpc1Controller<double>;

} // namespace ibvs_mpc1
