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


#include <ibvs_mpc1/ibvs_mpc1_controller.h>
#include <ibvs_mpc1/ibvs_mpc1_params.h>

// #include "autopilot/autopilot.h"

// template class autopilot::AutoPilot<ibvs_mpc::MpcController<float>,
//                          ibvs_mpc::MpcParams<float>>;
// template class autopilot::AutoPilot<ibvs_mpc::MpcController<double>,
//                          ibvs_mpc::MpcParams<double>>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ibvs1_autopilot");
  // autopilot::AutoPilot<ibvs_mpc::MpcController<float>,
  //                      ibvs_mpc::MpcParams<float>> autopilot;

  ros::spin();

  return 0;
}
