#ifndef IBVS_CIRCLE_H
#define IBVS_CIRCLE_H

// C++ libraries
#include <math.h>
#include <time.h>
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <circle_detection/CircleDetection.h>

// ROS libraries
#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// OpenCV libraries
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

// Custom libraries 
#include "vc_state/vc_state.h"

// Custom msgs 
#include "ibvs_pkg/Marker.h"
#include "ibvs_pkg/xyzyawVel.h"
#include "ibvs_pkg/point_xyz.h"
using namespace cv;
using namespace std;
namespace ibvs_pkg{
    class ibvsCircle{
        public:
            ibvsCircle(ros::NodeHandle nh);
            ~ibvsCircle();
            void imageCallback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg);
            // void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
            void quadCallback(const sensor_msgs::Imu::ConstPtr &msg);
            cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
            vector<Eigen::Vector3d> alignDepth2Color(Mat color, Mat depth, vector<Eigen::Vector3d> pd_uv, vc_parameters &params);
        private:
            ros::NodeHandle n;
            circle_detection::CircleDetection circleDetection;
            void init(ros::NodeHandle &nh);
            void initCamTopics(ros::NodeHandle &nh) {
                nh.param<string>("d435i_depth_topic", depth_topic, "/d435/depth/image_raw");
                nh.param<string>("d435i_color_topic", color_topic, "/d435/color/image_raw");
                color_sub_.subscribe(nh, color_topic, 1);
                depth_sub_.subscribe(nh, depth_topic, 1);
                cam_sync.connectInput(color_sub_, depth_sub_);
                cam_sync.registerCallback(boost::bind(&ibvsCircle::imageCallback, this, _1, _2));
            }
            int contIMG;
            // ros topics
            ros::Subscriber quad_sub, pos_sub;
            ros::Publisher puvz_pub, vel_pub, target_reached_pub;

            // message filter topics
            message_filters::Subscriber<sensor_msgs::Image> color_sub_, depth_sub_;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;
            message_filters::Synchronizer<mySyncPolicy> cam_sync{mySyncPolicy(10)};

            // ros messages
            std::string depth_topic, color_topic;
            std::string workspace, color_dir, depth_dir;
            sensor_msgs::ImagePtr image_msg;
            ibvs_pkg::Marker marker_msg;
            ibvs_pkg::xyzyawVel vel_msg;
            ibvs_pkg::point_xyz puvz_msg;
            std_msgs::Bool target_reached_msg;

            // Mat
            cv::Mat desired_depth, desired_color;
            cv::Mat img_old, img_points;

            // custom msgs
            vc_state state;
            vc_homograpy_matching_result matching_result;
            pair<Eigen::Vector3d, Eigen::Vector2d> ring_desired;
            pair<Eigen::Vector3d, Eigen::Vector2d> ring_actual;
    };
}

#endif