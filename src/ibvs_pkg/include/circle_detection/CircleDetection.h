#include "circle_detection/EDLib.h"
#include <iostream>
#include <string.h>
#include <time.h>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include "custom_msgs/CircleRecog.h"

// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include<opencv2/imgproc/imgproc.hpp>  

// c++
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <utility>

//Eigen
#include "Eigen/Core"
#include "Eigen/Dense"

namespace circle_detection{
    class CircleDetection{
        public:
            CircleDetection(ros::NodeHandle nh);
            ~CircleDetection();
            // Detect Result
            void detectRing(cv::Mat depth_buf, bool desired = false);
            std::pair<Eigen::Vector3d, Eigen::Vector2d> getResult(){
                return ring_result;
            }
        private:
            std::string depth_topic;
            ros::NodeHandle n;
            ros::Subscriber sub_depthimg;
            ros::Publisher pub_recg;
            cv::Mat depth_buf;
            const double camera_fx_ = 268.5;
            const double camera_cx_ = 320;
            const double camera_cy_ = 240;
            Eigen::Matrix3d camera_K_;
            Eigen::Matrix3d camera_K_inv_;
            void init();

            std::pair<Eigen::Vector3d, Eigen::Vector2d> ring_result;

            //Image Process
            cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
            void RemoveFarAwayPoints(cv::Mat src);
            cv::Mat CircleFloat2Int(cv::Mat src);
            cv::Mat EqualizeImg(cv::Mat src);

            // //for Depth Detection
            void getDepth2Circles(const std::vector<mCircle>&circles);
            void getDepth1Circle1Ellipse(const std::vector<mCircle> &circles, const std::vector<mEllipse> &ellipses);
            void getDepth2Ellipses(const std::vector<mEllipse> &ellipses);

            // //utils
            bool InsideEllipse(const mEllipse &ellipse,float ptx, float pty);
            bool InsideCircle(const mCircle &circle, float ptx, float pty);
            bool OutsideCircle(const mCircle &circle, float ptx, float pty);
            Eigen::Vector3d FromImage2CameraFrame(const Eigen::Vector3d &v1);



            // Eigen::Vector3d crossProduct(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
            //Publisher
            void PublishResult(const Eigen::Vector3d &CenterPt);
    };
}