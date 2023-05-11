/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <circle_detection/CircleDetection.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "circle_detection");
  ros::NodeHandle nodeHandle("~");
  circle_detection::CircleDetection circledetection(nodeHandle);

  ros::spin();

  ROS_INFO("hhh");
  return 0;
}
