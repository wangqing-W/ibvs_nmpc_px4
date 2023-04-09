#ifndef IBVS_DESCRIPTION_H
#define IBVS_DESCRIPTION_H

/******************************************************* ROS libraries*/
#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

/**************************************************** OpenCv Libraries*/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

/*************************************************** c++ libraries */
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <boost/thread/thread.hpp>

/*************************************************** custom libraries */
#include "vc_state/vc_state.h"

/*************************************************** custom msgs */
#include "ibvs_pkg/Marker.h"
#include "ibvs_pkg/xyzyawVel.h"
#include "ibvs_pkg/point_xyz.h"
#endif
