#include "ibvs_aruco.h"
#include <opencv2/aruco.hpp>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
/* Declaring namespaces */
using namespace cv;
using namespace std;

/* Declaring callbacks and other functions*/
void imageCallback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg);
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void quadCallback(const sensor_msgs::Imu::ConstPtr &msg);
cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
vector<Eigen::Vector3d> alignColor2Depth(Mat color, Mat depth, vector<Eigen::Vector3d> pc_uv, vc_parameters &params);
/* Declaring objects to receive messages */
sensor_msgs::ImagePtr image_msg;
ibvs_pkg::Marker marker_msg;
ibvs_pkg::xyzyawVel vel_msg;
ibvs_pkg::point_xyz puvz_msg;

/* Workspace definition from CMake */
// string workspace = WORKSPACE;
string workspace = "/home/zyh/ibvs_rpg_ws";

// Visual control state and results of the matching operation
vc_state state;
vc_homograpy_matching_result matching_result;

// Conteo de imágenes
int contIMG = 0, contGEN = 0, SAVE_DESIRED_POS, SAVE_IMAGES;

// Matrices para mostrar las imágenes
Mat img_old, img_points;

// define syncPolicy
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

/* Main function */
int main(int argc, char **argv)
{
    /***************************************************************************************** INIT */
	ros::init(argc, argv, "ibvs_aruco_node");
	ros::NodeHandle nh;
	state.load(nh);
    nh.getParam("SAVE_DESIRED_POS", SAVE_DESIRED_POS);
	nh.getParam("SAVE_IMAGES", SAVE_IMAGES);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;
	message_filters::Subscriber<sensor_msgs::Image> subscriber_color(nh,"/d435/color/image_raw",10,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth(nh,"/d435/depth/image_raw",10,ros::TransportHints().tcpNoDelay());

	ros::Subscriber quad_sub;
	ros::Publisher puvz_pub;
	quad_sub = nh.subscribe("/hummingbird/imu", 1, &quadCallback, ros::TransportHints().tcpNoDelay());
	puvz_pub = nh.advertise<ibvs_pkg::point_xyz>("/hummingbird/markerpoint", 1);
    /************************************************************* CREATING PUBLISHER AND SUBSCRIBER */
	string image_dir;
    cout << "[INFO] Using realsense d435i camera" << endl;
    //message_filters::TimeSynchronizer<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped> sync(subscriber_laser, subscriber_pose, 10);
    message_filters::Synchronizer<syncPolicy> cam_sync(syncPolicy(10), subscriber_color, subscriber_depth);  
    cam_sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    image_dir = "/src/ibvs_pkg/src/target/desired_f.png";
    image_transport::Publisher image_pub = it.advertise("matching", 1);
	ros::Rate rate(30);

    /************************************************************************** OPENING DESIRED IMAGE */

	state.desired_configuration.img = imread(workspace + image_dir, IMREAD_COLOR);
	if (state.desired_configuration.img.empty())
	{
		cerr << "[ERROR] Could not open or find the reference image" << std::endl;
		cout << "[ERROR] No dir >> " << workspace + image_dir << endl;
		exit(-1);
	}
	else
	{
		cout << "[INFO] Reference image loaded" << std::endl;
	}

    Ptr<ORB> orb = ORB::create(state.params.nfeatures,
														 state.params.scaleFactor,
														 state.params.nlevels,
														 state.params.edgeThreshold,
														 state.params.firstLevel,
														 state.params.WTA_K,
														 state.params.scoreType,
														 state.params.patchSize,
														 state.params.fastThreshold);

	orb->detect(state.desired_configuration.img,
							state.desired_configuration.kp);
	orb->compute(state.desired_configuration.img,
							 state.desired_configuration.kp,
							 state.desired_configuration.descriptors);

    /******************************************************************************* MOVING TO A POSE */
	ros::Publisher pos_pub;
	ros::Subscriber pos_sub;

    cout << "[INFO] Hummingbird trajectory and pose" << endl
                << endl;
    pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 1);
    pos_sub = nh.subscribe<geometry_msgs::Pose>("/hummingbird/ground_truth/pose", 1, poseCallback);
	ros::Publisher target_reached_pub = nh.advertise<std_msgs::Bool>("target_reached", 1);
	/******************************************************************************* DETECT TO A POSE */
	ros::Publisher marker_pub, vel_pub, est_vel_pub;
	marker_pub = nh.advertise<ibvs_pkg::Marker>("/hummingbird/camera/Marker", 1);
	vel_pub = nh.advertise<ibvs_pkg::xyzyawVel>("/hummingbird/reference_vel", 1);
	est_vel_pub = nh.advertise<ibvs_pkg::xyzyawVel>("/hummingbird/estimation_vel", 1);
	marker_msg.header.frame_id = "marker_position";
	vel_msg.header.frame_id = "reference_velocity";
	puvz_msg.header.frame_id = "marker_uvz";
	/**************************************************************************** data for graphics */
	vector<float> vel_x;
	vector<float> vel_y;
	vector<float> vel_z;
	vector<float> vel_yaw;
	vector<float> errors;
	vector<float> errors_pix;
	vector<float> time;
	vector<float> lambda;

	// Create message for the pose
	trajectory_msgs::MultiDOFJointTrajectory msg;
	string file_folder = "/src/ibvs_pkg/src/data/";

	/******************************************************************************* CYCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();

		if (contGEN > 50 && SAVE_DESIRED_POS)
		{
			cout << "\n[INFO] Images have been saved." << endl;
			ros::shutdown();
		}

		if (!state.initialized)
		{
			rate.sleep();
			continue;
		} // if we havent get the new pose

		if (matching_result.mean_feature_error < state.params.feature_threshold)
		{
			cout << endl
					 << "[INFO] Target reached within the feature threshold and maximum iterations" << endl
					 << endl;
			std_msgs::Bool target_reached_msg;
			target_reached_msg.data = true;
			target_reached_pub.publish(target_reached_msg);
			waitKey(0);
			break;
		}
		else
		{
			contGEN++;
		}

		// Publish image of the matching
		cout << endl
				 << "[INFO] Publishing image" << endl;
		image_pub.publish(image_msg);
		// marker_pub.publish(marker_msg);
		// 使用swap,清除元素并回收内存
    	// vector <int>().swap(marker_msg.marker_desired_x);  //清除容器并最小化它的容量，
		// vector <int>().swap(marker_msg.marker_desired_y);  //清除容器并最小化它的容量，
		// vector <int>().swap(marker_msg.marker_current_x);  //清除容器并最小化它的容量，
		// vector <int>().swap(marker_msg.marker_current_y);  //清除容器并最小化它的容量，
		vel_msg.header.stamp = ros::Time::now();
		Eigen::Vector3d VelB(state.Vx, state.Vy, state.Vz);
		Eigen::Vector3d VelW;
		VelW = state.quad * VelB;
		vel_msg.Vx = VelW.x();
		vel_msg.Vy = VelW.y();
		vel_msg.Vz = VelW.z();
		vel_msg.Vyaw = state.Vyaw;
		vel_pub.publish(vel_msg);

		puvz_msg.header.stamp = ros::Time::now();
		puvz_pub.publish(puvz_msg);
		// // save data
		// time.push_back(state.t);
		// errors.push_back((float)matching_result.mean_feature_error);
		// errors_pix.push_back((float)matching_result.mean_feature_error_pix);
		// vel_x.push_back(state.Vx);
		// vel_y.push_back(state.Vy);
		// vel_z.push_back(state.Vz);
		// vel_yaw.push_back(state.Vyaw);
		// lambda.push_back(state.lambda);

		// // Update state with the current control
		// auto new_pose = state.update();

		// vel_yaw.push_back(state.Vyaw);

		// // Prepare msg
		// msg.header.stamp = ros::Time::now();
		// mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

		// // Publish msg
		// pos_pub.publish(msg);
		rate.sleep();

		if (contIMG > 1000)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			break;
		}
	}

	// save data
	// writeFile(errors, workspace + file_folder + "errors.txt");
	// writeFile(errors_pix, workspace + file_folder + "errors_pix.txt");
	// writeFile(time, workspace + file_folder + "time.txt");
	// writeFile(vel_x, workspace + file_folder + "Vx.txt");
	// writeFile(vel_y, workspace + file_folder + "Vy.txt");
	// writeFile(vel_z, workspace + file_folder + "Vz.txt");
	// writeFile(vel_yaw, workspace + file_folder + "Vyaw.txt");
	// writeFile(lambda, workspace + file_folder + "lambda.txt");

	return 0;
}

/*
	Function: align_Color2Depth
	description: align the color image to depth image
	params:
	 color, depth: the rgb and depth image from d435i
	 pc_uv: the points to be aligned in color image
	 params: it contains the intrinsic param of color and depth
	return:
	 pd_uv: the points aligned in depth image
*/
vector<Eigen::Vector3d> alignColor2Depth(Mat color, Mat depth, vector<Eigen::Vector3d> pc_uv, vc_parameters &params){
	//利用Eigen存放内参
    Eigen::Matrix3d intrinColor_matrix;
    intrinColor_matrix<<params.K.at<double>(0, 0), params.K.at<double>(0, 1), params.K.at<double>(0, 2),
						params.K.at<double>(1, 0), params.K.at<double>(1, 1), params.K.at<double>(1, 2),
						params.K.at<double>(2, 0), params.K.at<double>(2, 1), params.K.at<double>(2, 2);
    Eigen::Matrix3d intrinDepth_matrix;
    intrinDepth_matrix<<params.K_depth.at<double>(0, 0), params.K_depth.at<double>(0, 1), params.K_depth.at<double>(0, 2),
						params.K_depth.at<double>(1, 0), params.K_depth.at<double>(1, 1), params.K_depth.at<double>(1, 2),
						params.K_depth.at<double>(2, 0), params.K_depth.at<double>(2, 1), params.K_depth.at<double>(2, 2);
	
	//Eigen 用3*3矩阵表示旋转矩阵
    Eigen::Matrix3d R_matrix;
    R_matrix<< 1,0,0,
               0,1,0,
               0,0,1;
 
    //Eigen 用三维向量表示平移量
    Eigen::Vector3d t_vector;
    t_vector<<0, 0, 0;
	
	//Eigen 声明欧氏变换转移矩阵T,用上面的旋转矩阵和平移量构造
    Eigen::Isometry3d T_Color2Depth=Eigen::Isometry3d::Identity();
    T_Color2Depth.rotate(R_matrix);
    T_Color2Depth.pretranslate(t_vector);

	//获取深度像素与现实单位比例（D435默认1毫米）
    // float depth_scale = 1/1000.0;

	//初始化结果
	int num = pc_uv.size();
	vector<Eigen::Vector3d> pd_uv(num);

	Eigen::Vector3d pcc3;//彩色坐标系坐标
	//对RGB图待对齐点遍历
	for(int i=0; i<num; i++){
		pcc3[0]=(pc_uv[i][0]-intrinColor_matrix(0,2))/intrinColor_matrix(0,0);
		pcc3[1]=(pc_uv[i][1]-intrinColor_matrix(1,2))/intrinColor_matrix(1,1);
		pcc3[2]=1;
		pd_uv[i] = intrinDepth_matrix * T_Color2Depth * pcc3;
	}
    
    return pd_uv;
}

/*
	Function: getDepthImageFromMsg
	description: get the depth image in the format of cv_bridge::CvImagePtr from d435i
	params: message with depth image from d435i
*/
cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    //depth has encoding TYPE_32FC1
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

    if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000);
    }
    cv::Mat depth;
    depth = cv_ptr->image.clone();

    return depth;
}

/*
	Function: PoseCallback
	description: get the ppose info from the groundtruth of the drone and uses it in simulation
	params: message with pose info
*/
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	state.Roll = (float)roll;
	state.Pitch = (float)pitch;
	state.Yaw = (float)yaw;
	// setting the position if its the first time
	if (!state.initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		state.initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}
}

void quadCallback(const sensor_msgs::Imu::ConstPtr &msg){
	state.quad.w() = msg->orientation.w;
	state.quad.x() = msg->orientation.x;
	state.quad.y() = msg->orientation.y;
	state.quad.z() = msg->orientation.z;
	// cout << "quad:" << state.quad.w() << ", " << state.quad.x() << ", " << state.quad.y() << ", " << state.quad.z() << endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
	cout << "[INFO] ImageCallback function" << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(color_msg, "bgr8")->image;
		Mat actual_depth = getDepthImageFromMsg(depth_msg);
		// resize(actual_depth,depth_actual,Size(actual.cols, actual.rows),0,0,INTER_LINEAR);
		cout << "[INFO] Image received" << endl;

		if (contIMG < 3)
		{
			contIMG++;
			cout << endl
					 << "[INFO] Detecting keypoints" << endl;

			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
			cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::aruco::detectMarkers(actual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

			Mat temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p2, CV_64F);
			temporal.convertTo(img_points, CV_32F);

			cv::aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p1, CV_64F);

			cout << "[INFO] img_points: " << img_points << endl;
			cout << "[INFO] matching_result.p1: " << matching_result.p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_result.p2 << endl;

			img_old = actual;
		}
		else
		{
			Mat img_new = actual;

			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
			cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::aruco::detectMarkers(actual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

			Mat temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			// cout << "markerCorners_x:" << markerCorners[0][0].x << "," << markerCorners[0][1].x << "," 
			// << markerCorners[0][2].x << "," << markerCorners[0][3].x << endl;
			vector<Eigen::Vector3d> pc_uv, pd_uv;
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][0].x, markerCorners[0][0].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][1].x, markerCorners[0][1].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][2].x, markerCorners[0][2].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][3].x, markerCorners[0][3].y, 1));
			pd_uv = alignColor2Depth(actual, actual_depth, pc_uv, state.params);
			// cout << "pd_uv in depth image:" << pd_uv[0][0] << "," << pd_uv[0][1] << endl
			// 								<< pd_uv[1][0] << "," << pd_uv[1][1] << endl
			// 								<< pd_uv[2][0] << "," << pd_uv[2][1] << endl
			// 								<< pd_uv[3][0] << "," << pd_uv[3][1] << endl;	
			vector<float> depthVec{(float)actual_depth.at<unsigned short>(pd_uv[0][1], pd_uv[0][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[1][1], pd_uv[1][0]) /1000,
									(float)actual_depth.at<unsigned short>(pd_uv[2][1], pd_uv[2][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[3][1], pd_uv[3][0]) /1000};
			std::cout << "Marker pixel values: " << depthVec[0] 
			<< ", " << depthVec[1] 
			<< ", " << depthVec[2] 
			<< ", " << depthVec[3] 
			<< std::endl;
			puvz_msg.p1_x = (pc_uv[0].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p1_y = (pc_uv[0].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p1_z = depthVec[0];
			puvz_msg.p2_x = (pc_uv[1].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p2_y = (pc_uv[1].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p2_z = depthVec[1];
			puvz_msg.p3_x = (pc_uv[2].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p3_y = (pc_uv[2].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p3_z = depthVec[2];
			puvz_msg.p4_x = (pc_uv[3].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p4_y = (pc_uv[3].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p4_z = depthVec[3];

			temporal.convertTo(matching_result.p2, CV_64F);
			temporal.convertTo(img_points, CV_32F);

			cv::aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p1, CV_64F);
			
			// AQUI ES DONDE SE EJECUTA TODOOOOO
			if (GUO(actual, depthVec, state, matching_result) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			// KLT tracker for the next iteration
			Mat new_points, status, error;
			Mat img_old_gray, img_new_gray;
			cvtColor(img_old, img_old_gray, COLOR_BGR2GRAY);
			cvtColor(img_new, img_new_gray, COLOR_BGR2GRAY);
			calcOpticalFlowPyrLK(img_old_gray, img_new_gray, img_points, new_points, status, error);

			Mat desired_temp = state.desired_configuration.img.clone();
			for (int i = 0; i < matching_result.p1.rows; i++)
			{
				circle(desired_temp, Point2f(matching_result.p1.at<double>(i, 0), matching_result.p1.at<double>(i, 1)), 3, Scalar(0, 0, 255), -1);//RED
			}
			for (int i = 0; i < new_points.rows; i++)
			{
				vector<Eigen::Vector3d> c1_uv, c2_uv, d1_uv, d2_uv;
				c1_uv.push_back(Eigen::Vector3d(img_points.at<Point2f>(i, 0).x, img_points.at<Point2f>(i, 0).y, 1));
				c2_uv.push_back(Eigen::Vector3d(new_points.at<Point2f>(i, 0).x, new_points.at<Point2f>(i, 0).y, 1));
				d1_uv = alignColor2Depth(actual, actual_depth, c1_uv, state.params);
				d2_uv = alignColor2Depth(actual, actual_depth, c2_uv, state.params);
				circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
				circle(actual, new_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);// Red
				circle(actual, img_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
				// cout << "coordinates in depth image:" << d2_uv[0][0] << "," << d2_uv[0][1] << endl;				  
				circle(actual_depth, Point2f(d1_uv[0][0], d1_uv[0][1]), 6, Scalar(255, 255, 255), -1);
				circle(actual_depth, Point2f(d2_uv[0][0], d2_uv[0][1]), 6, Scalar(255, 255, 255), -1);
			}
			imshow("Desired", desired_temp);
			imshow("Cámara color", actual);
			imshow("Cámara depth", actual_depth);
			waitKey(1);

			new_points.convertTo(matching_result.p2, CV_64F);
			img_points = new_points.clone();
			actual.copyTo(img_old);
		}

		// if (SAVE_IMAGES)
		// {
		// 	string saveIMG = "/src/vc_new_controller/src/data/img/" + to_string(contIMG++) + ".jpg";
		// 	imwrite(workspace + saveIMG, actual);
		// 	cout << "[INFO] << Image saved >>" << saveIMG << endl;
		// }
		// else
		// {
		// 	string saveIMG = "/src/vc_new_controller/src/data/img/0.jpg";
		// 	imwrite(workspace + saveIMG, actual);
		// 	cout << "[INFO] << Image saved >>" << saveIMG << endl;
		// }

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_result.img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_result.img_matches.cols;
		image_msg->height = matching_result.img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_result.img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (state.initialized)
			cout << "[VELS] Vx: " << state.Vx << ", Vy: " << state.Vy << ", Vz: " << state.Vz << "\nVroll: " << state.Vroll << ", Vpitch: " << state.Vpitch << ", Wyaw: " << state.Vyaw << "\n==> average error: " << matching_result.mean_feature_error << "<==" << endl
					 << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
							image_msg->encoding.c_str());
	}
}

void imageCallback2(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
	cout << "[INFO] ImageCallback function" << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(color_msg, "bgr8")->image;
		Mat actual_depth = getDepthImageFromMsg(depth_msg);
		// resize(actual_depth,depth_actual,Size(actual.cols, actual.rows),0,0,INTER_LINEAR);
		cout << "[INFO] Image received" << endl;

		if (contIMG < 3)
		{
			contIMG++;
			cout << endl
					 << "[INFO] Detecting keypoints" << endl;

			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
			// cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
			// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			// cv::aruco::detectMarkers(actual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			// TODO:: detect the red circle in the actual image

			Mat temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p2, CV_64F);
			temporal.convertTo(img_points, CV_32F);

			// TODO:: detect the red circle in the desired image
			// cv::aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p1, CV_64F);

			cout << "[INFO] img_points: " << img_points << endl;
			cout << "[INFO] matching_result.p1: " << matching_result.p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_result.p2 << endl;

			img_old = actual;
		}
		else
		{
			Mat img_new = actual;

			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
			// cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
			// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			// cv::aruco::detectMarkers(actual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			// TODO:: detect the red circle in the actual image

			Mat temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			// cout << "markerCorners_x:" << markerCorners[0][0].x << "," << markerCorners[0][1].x << "," 
			// << markerCorners[0][2].x << "," << markerCorners[0][3].x << endl;
			vector<Eigen::Vector3d> pc_uv, pd_uv;
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][0].x, markerCorners[0][0].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][1].x, markerCorners[0][1].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][2].x, markerCorners[0][2].y, 1));
			pc_uv.push_back(Eigen::Vector3d(markerCorners[0][3].x, markerCorners[0][3].y, 1));
			pd_uv = alignColor2Depth(actual, actual_depth, pc_uv, state.params);
			// cout << "pd_uv in depth image:" << pd_uv[0][0] << "," << pd_uv[0][1] << endl
			// 								<< pd_uv[1][0] << "," << pd_uv[1][1] << endl
			// 								<< pd_uv[2][0] << "," << pd_uv[2][1] << endl
			// 								<< pd_uv[3][0] << "," << pd_uv[3][1] << endl;	
			vector<float> depthVec{(float)actual_depth.at<unsigned short>(pd_uv[0][1], pd_uv[0][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[1][1], pd_uv[1][0]) /1000,
									(float)actual_depth.at<unsigned short>(pd_uv[2][1], pd_uv[2][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[3][1], pd_uv[3][0]) /1000};
			std::cout << "Marker pixel values: " << depthVec[0] 
			<< ", " << depthVec[1] 
			<< ", " << depthVec[2] 
			<< ", " << depthVec[3] 
			<< std::endl;
			puvz_msg.p1_x = (pc_uv[0].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p1_y = (pc_uv[0].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p1_z = depthVec[0];
			puvz_msg.p2_x = (pc_uv[1].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p2_y = (pc_uv[1].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p2_z = depthVec[1];
			puvz_msg.p3_x = (pc_uv[2].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p3_y = (pc_uv[2].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p3_z = depthVec[2];
			puvz_msg.p4_x = (pc_uv[3].x() - state.params.K.at<double>(0, 2))/state.params.K.at<double>(0, 0);
			puvz_msg.p4_y = (pc_uv[3].y() - state.params.K.at<double>(1, 2))/state.params.K.at<double>(1, 1);
			puvz_msg.p4_z = depthVec[3];

			temporal.convertTo(matching_result.p2, CV_64F);
			temporal.convertTo(img_points, CV_32F);

			// TODO:: detect the red circle in the desired image
			// cv::aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
			temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(markerCorners[0][0].x, markerCorners[0][0].y);
			temporal.at<Point2f>(1, 0) = Point2f(markerCorners[0][1].x, markerCorners[0][1].y);
			temporal.at<Point2f>(2, 0) = Point2f(markerCorners[0][2].x, markerCorners[0][2].y);
			temporal.at<Point2f>(3, 0) = Point2f(markerCorners[0][3].x, markerCorners[0][3].y);
			temporal.convertTo(matching_result.p1, CV_64F);
			
			// AQUI ES DONDE SE EJECUTA TODOOOOO
			if (GUO(actual, depthVec, state, matching_result) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			// KLT tracker for the next iteration
			Mat new_points, status, error;
			Mat img_old_gray, img_new_gray;
			cvtColor(img_old, img_old_gray, COLOR_BGR2GRAY);
			cvtColor(img_new, img_new_gray, COLOR_BGR2GRAY);
			calcOpticalFlowPyrLK(img_old_gray, img_new_gray, img_points, new_points, status, error);

			Mat desired_temp = state.desired_configuration.img.clone();
			for (int i = 0; i < matching_result.p1.rows; i++)
			{
				circle(desired_temp, Point2f(matching_result.p1.at<double>(i, 0), matching_result.p1.at<double>(i, 1)), 3, Scalar(0, 0, 255), -1);//RED
			}
			for (int i = 0; i < new_points.rows; i++)
			{
				vector<Eigen::Vector3d> c1_uv, c2_uv, d1_uv, d2_uv;
				c1_uv.push_back(Eigen::Vector3d(img_points.at<Point2f>(i, 0).x, img_points.at<Point2f>(i, 0).y, 1));
				c2_uv.push_back(Eigen::Vector3d(new_points.at<Point2f>(i, 0).x, new_points.at<Point2f>(i, 0).y, 1));
				d1_uv = alignColor2Depth(actual, actual_depth, c1_uv, state.params);
				d2_uv = alignColor2Depth(actual, actual_depth, c2_uv, state.params);
				circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
				circle(actual, new_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);// Red
				circle(actual, img_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
				// cout << "coordinates in depth image:" << d2_uv[0][0] << "," << d2_uv[0][1] << endl;				  
				circle(actual_depth, Point2f(d1_uv[0][0], d1_uv[0][1]), 6, Scalar(255, 255, 255), -1);
				circle(actual_depth, Point2f(d2_uv[0][0], d2_uv[0][1]), 6, Scalar(255, 255, 255), -1);
			}
			imshow("Desired", desired_temp);
			imshow("Cámara color", actual);
			imshow("Cámara depth", actual_depth);
			waitKey(1);

			new_points.convertTo(matching_result.p2, CV_64F);
			img_points = new_points.clone();
			actual.copyTo(img_old);
		}

		// if (SAVE_IMAGES)
		// {
		// 	string saveIMG = "/src/vc_new_controller/src/data/img/" + to_string(contIMG++) + ".jpg";
		// 	imwrite(workspace + saveIMG, actual);
		// 	cout << "[INFO] << Image saved >>" << saveIMG << endl;
		// }
		// else
		// {
		// 	string saveIMG = "/src/vc_new_controller/src/data/img/0.jpg";
		// 	imwrite(workspace + saveIMG, actual);
		// 	cout << "[INFO] << Image saved >>" << saveIMG << endl;
		// }

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_result.img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_result.img_matches.cols;
		image_msg->height = matching_result.img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_result.img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (state.initialized)
			cout << "[VELS] Vx: " << state.Vx << ", Vy: " << state.Vy << ", Vz: " << state.Vz << "\nVroll: " << state.Vroll << ", Vpitch: " << state.Vpitch << ", Wyaw: " << state.Vyaw << "\n==> average error: " << matching_result.mean_feature_error << "<==" << endl
					 << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
							image_msg->encoding.c_str());
	}
}
