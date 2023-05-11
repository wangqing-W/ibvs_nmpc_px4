#ifndef VC_STATE_H
#define VC_STATE_H
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "vc_state/img_tools.h"
// #include "vc_state/math_custom.h"

// # Lee position controller parameters
// position_gain: {x: 6, y: 6, z: 6}
// velocity_gain: {x: 4.7, y: 4.7, z: 4.7}
// attitude_gain: {x: 2, y: 3, z: 0.15}
// angular_rate_gain: {x: 0.4, y: 0.52, z: 0.18}

using namespace std;

typedef struct vecDist
{
        int i;        // index i
        int j;        // index j p_i * p_j
        double dist;  // distance between p2_i and p2_j
        double dist2; // distance between p1_i and p1_j
} vecDist;

class vc_state {
	public:
		/* defining where the drone will move and integrating system*/
		float X= 0.0,Y= 0.0,Z= 0.0,Yaw= 0.0,Pitch= 0.0,Roll= 0.0;
        Eigen::Quaterniond quad;
		bool initialized=false;
// 		float t,dt;
// 		float Kv,Kw;
        
        /* Control parameters  */
        float Vx=0.0,Vy=0.0,Vz=0.0;
        float Vx_est=0.0, Vy_est=0.0, Vz_est=0.0;
        float Vyaw=0.0, Vroll = 0.0, Vpitch = 0.0;
        float Kv=1.0;
        float Kw=1.0;
        float dt=0.025;
        float t=0;
        float lambda = 0;

        // Image proessing parameters
        vc_parameters params;
        //  Desired configuration
        vc_desired_configuration desired_configuration;
        
        //  Best approximations
        bool selected = false;
        cv::Mat t_best;
        cv::Mat R_best; //to save the rot and trans
        
        /* ROS pub */
		ros::Publisher ros_pub;
		
        // Methods
		vc_state();

        std::pair<Eigen::VectorXd,float> update();
        void load(const ros::NodeHandle &nh);
		void initialize(const float &x,
                        const float &y,
                        const float &z,
                        const float &yaw);
//         int (*controller )[1];
};



//  Controllers

// int homography(cv::Mat img,
//                vc_state & state,
//                vc_homograpy_matching_result & matching_result
//               );

// int chaumette(cv::Mat img,
//                vc_state & state,
//                vc_homograpy_matching_result & matching_result
//               );

int GUO(cv::Mat img,
        vector<float> depthVec,
               vc_state & state,
               vc_homograpy_matching_result & matching_result
              );

// Controller selection array only for vc_controller.h
#ifdef VC_CONTROLLER_H
typedef int (*funlist) (cv::Mat img,
                        vector<float> depthVec,
                        vc_state & state,
                        vc_homograpy_matching_result & matching_result
                       );
funlist controllers[] = {&homography, &chaumette, &GUO};
#endif

#endif
