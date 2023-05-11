#include <ros/ros.h>
#include <ibvs_circle.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "ibvs_circle_node");
  ros::NodeHandle nodeHandle;
  ibvs_pkg::ibvsCircle ibvsCircle(nodeHandle);

  ros::spin();

  return 0;
}