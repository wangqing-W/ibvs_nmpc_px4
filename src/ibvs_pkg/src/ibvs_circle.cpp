#include <ibvs_circle.h>
#define REALRACE 0
using namespace cv;
using namespace std;

namespace ibvs_pkg{
ibvsCircle::ibvsCircle(ros::NodeHandle nh):n(nh), circleDetection(nh){
    init(nh);
}

ibvsCircle::~ibvsCircle(){};

void ibvsCircle::init(ros::NodeHandle &nh)
{
    contIMG = 0;
    state.load(nh);
    initCamTopics(nh);
    workspace = "/home/drone/zyh/ibvs_nmpc_px4_ws";
    color_dir = "/src/ibvs_pkg/target/desired_c.png";
    depth_dir = "/src/ibvs_pkg/target/desired_d.png";
    quad_sub = nh.subscribe("/mavros/imu/data", 1, &ibvsCircle::quadCallback, this, ros::TransportHints().tcpNoDelay());
    // pos_sub = nh.subscribe("/hummingbird/ground_truth/pose", 1, &ibvsCircle::poseCallback, this, ros::TransportHints().tcpNoDelay());
    target_reached_pub = nh.advertise<std_msgs::Bool>("ring/target_reached", 1);
	puvz_pub = nh.advertise<ibvs_pkg::point_xyz>("/hummingbird/markerpoint", 1);
    vel_pub = nh.advertise<ibvs_pkg::xyzyawVel>("/hummingbird/reference_vel", 1);
    vel_msg.header.frame_id = "reference_velocity";
	puvz_msg.header.frame_id = "marker_uvz";
    target_reached_msg.data = false;
    desired_depth = imread(workspace + depth_dir, IMREAD_UNCHANGED);
    desired_color = imread(workspace + color_dir, IMREAD_COLOR);
    state.desired_configuration.img = desired_color;
	if (desired_color.empty() || desired_depth.empty())
	{
		cerr << "[ERROR] Could not open or find the reference image" << std::endl;
		cout << "[ERROR] No dir >> " << workspace + color_dir << endl;
		exit(-1);
	}
	else
	{
		cout << "[INFO] Reference image loaded" << std::endl;
	}
}

void ibvsCircle::imageCallback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
	ROS_INFO("[INFO] ImageCallback function");
    Mat actual = cv_bridge::toCvShare(color_msg, "bgr8")->image;
    Mat actual_depth = getDepthImageFromMsg(depth_msg);
    // resize(actual_depth,depth_actual,Size(actual.cols, actual.rows),0,0,INTER_LINEAR);
    ROS_INFO("[INFO] Image received");
    // 如果处于中间阶段
    if(target_reached_msg.data)
    {
        ROS_INFO("middle stage!!");
        bool flag;
        flag = circleDetection.detectRing(actual_depth, false);
        if(!flag) ROS_INFO("middle stage —— NOOOOO!!");
        else{
            ROS_INFO("middle stage —— YESSSS!!");
            ring_actual = circleDetection.getResult();
            ROS_INFO("ring_actual.first:%f, %f, %f", ring_actual.first[0], ring_actual.first[1], ring_actual.first[2]);
            if(ring_actual.first[2] > 2000/**/ && ring_actual.first[2] < 3500) target_reached_msg.data = false;
        }
        target_reached_pub.publish(target_reached_msg);
        return;
    }
    double ring_x, ring_y, ring_w, ring_h;
    if (contIMG < 3)
    {
        contIMG++;
        cout << endl
             << "[INFO] Detecting circles for initial 3 frames" << endl;
        //TODO: detect in the actual image
        circleDetection.detectRing(actual_depth, false);
        ring_actual = circleDetection.getResult();
        ring_x = ring_actual.first(0);
        ring_y = ring_actual.first(1);
        ring_w = ring_actual.second(0);
        ring_h = ring_actual.second(1);
        vector<Eigen::Vector3d> pc_uv, pd_uv;
        pd_uv.push_back(Eigen::Vector3d(ring_x, ring_y + ring_h, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x + ring_w, ring_y, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x, ring_y - ring_h, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x - ring_w, ring_y, 1));
        pc_uv = alignDepth2Color(actual, actual_depth, pd_uv, state.params);

        Mat temporal = Mat::zeros(4, 2, CV_32F);
        temporal.at<Point2f>(0, 0) = Point2f(pc_uv[0][0], pc_uv[0][1]);
        temporal.at<Point2f>(1, 0) = Point2f(pc_uv[1][0], pc_uv[1][1]);
        temporal.at<Point2f>(2, 0) = Point2f(pc_uv[2][0], pc_uv[2][1]);
        temporal.at<Point2f>(3, 0) = Point2f(pc_uv[3][0], pc_uv[3][1]);
        temporal.convertTo(matching_result.p2, CV_64F);
        temporal.convertTo(img_points, CV_32F);

        //TODO: detect in the desired image
        ROS_INFO("Detect desired!!!");
        circleDetection.detectRing(desired_depth, true);
        ring_desired = circleDetection.getResult();
        ring_x = ring_desired.first(0);
        ring_y = ring_desired.first(1);
        ring_w = ring_desired.second(0);
        ring_h = ring_desired.second(1);
        vector<Eigen::Vector3d> pc1_uv, pd1_uv;
        pd1_uv.push_back(Eigen::Vector3d(ring_x, ring_y + ring_h, 1));
        pd1_uv.push_back(Eigen::Vector3d(ring_x + ring_w, ring_y, 1));
        pd1_uv.push_back(Eigen::Vector3d(ring_x, ring_y - ring_h, 1));
        pd1_uv.push_back(Eigen::Vector3d(ring_x - ring_w, ring_y, 1));
        pc1_uv = alignDepth2Color(desired_color, desired_depth, pd1_uv, state.params);
        // Mat temporal = Mat::zeros(4, 2, CV_32F);
        temporal.at<Point2f>(0, 0) = Point2f(pc1_uv[0][0], pc1_uv[0][1]);
        temporal.at<Point2f>(1, 0) = Point2f(pc1_uv[1][0], pc1_uv[1][1]);
        temporal.at<Point2f>(2, 0) = Point2f(pc1_uv[2][0], pc1_uv[2][1]);
        temporal.at<Point2f>(3, 0) = Point2f(pc1_uv[3][0], pc1_uv[3][1]);
        temporal.convertTo(matching_result.p1, CV_64F);

        cout << "[INFO] img_points: " << img_points << endl;
        cout << "[INFO] matching_result.p1: " << matching_result.p1 << endl;
        cout << "[INFO] matching_result.p2: " << matching_result.p2 << endl;

        img_old = actual;
    }
    else
    {
        ros::Time t0 = ros::Time::now();
        double ring_time, align_first_time, align_second_time, pub_vel_time, control_time, draw_time;
        Mat img_new = actual;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        //TODO: detect in the actual image
        ROS_INFO("Detect actual!!!");
        circleDetection.detectRing(actual_depth, false);
        // ring_time = (ros::Time::now()-t0).toSec();
        // t0 = ros::Time::now();

        ring_actual = circleDetection.getResult();
        ring_x = ring_actual.first(0);
        ring_y = ring_actual.first(1);
        ring_w = ring_actual.second(0);
        ring_h = ring_actual.second(1);
        ROS_INFO("ring_w,h: %f, %f", ring_w, ring_h);
        vector<Eigen::Vector3d> pc_uv, pd_uv;
        pd_uv.push_back(Eigen::Vector3d(ring_x, ring_y + ring_h, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x + ring_w, ring_y, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x, ring_y - ring_h, 1));
        pd_uv.push_back(Eigen::Vector3d(ring_x - ring_w, ring_y, 1));
        pc_uv = alignDepth2Color(actual, actual_depth, pd_uv, state.params);
        // align_first_time = (ros::Time::now()-t0).toSec();
        // t0 = ros::Time::now();

        Mat temporal = Mat::zeros(4, 2, CV_32F);
        temporal.at<Point2f>(0, 0) = Point2f(pc_uv[0][0], pc_uv[0][1]);
        temporal.at<Point2f>(1, 0) = Point2f(pc_uv[1][0], pc_uv[1][1]);
        temporal.at<Point2f>(2, 0) = Point2f(pc_uv[2][0], pc_uv[2][1]);
        temporal.at<Point2f>(3, 0) = Point2f(pc_uv[3][0], pc_uv[3][1]);
        vector<float> depthVec{(float)actual_depth.at<unsigned short>(pd_uv[0][1], pd_uv[0][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[1][1], pd_uv[1][0]) /1000,
                                (float)actual_depth.at<unsigned short>(pd_uv[2][1], pd_uv[2][0]) /1000, (float)actual_depth.at<unsigned short>(pd_uv[3][1], pd_uv[3][0]) /1000};
        // std::cout << "Marker pixel values: " << depthVec[0] 
        // << ", " << depthVec[1] 
        // << ", " << depthVec[2] 
        // << ", " << depthVec[3]
        // << std::endl;
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

		puvz_msg.header.stamp = ros::Time::now();
		puvz_pub.publish(puvz_msg);
        // pub_vel_time = (ros::Time::now()-t0).toSec();
        // t0 = ros::Time::now();

        temporal.convertTo(matching_result.p2, CV_64F);
        temporal.convertTo(img_points, CV_32F);

        // cv::aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        //TODO: detect in the desired image
        // AQUI ES DONDE SE EJECUTA TODOOOOO
        if (GUO(actual, depthVec, state, matching_result) < 0)
        {
            ROS_ERROR("[ERROR] Controller failed");
            return;
        }
        else
        {
            vel_msg.header.stamp = ros::Time::now();
            Eigen::Vector3d VelB(state.Vx, state.Vy, state.Vz);
            Eigen::Vector3d VelW;
            VelW = state.quad * VelB;
            vel_msg.Vx = VelW.x();
            vel_msg.Vy = VelW.y();
            vel_msg.Vz = VelW.z();
            vel_msg.Vyaw = state.Vyaw;
            vel_pub.publish(vel_msg);
            ROS_INFO("Controller part has been executed");
        }
        // control_time = (ros::Time::now()-t0).toSec();
        // t0 = ros::Time::now();
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
            circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
            circle(actual, new_points.at<Point2f>(i, 0), 3, Scalar(0, 255, 0), -1);// Green
            circle(actual, img_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);// Blue
        }
        imshow("Desired", desired_temp);
        imshow("Cámara color", actual);
        // imshow("Cámara depth", actual_depth);
        waitKey(1);
        // draw_time = (ros::Time::now()-t0).toSec();
        // t0 = ros::Time::now();
        new_points.convertTo(matching_result.p2, CV_64F);
        img_points = new_points.clone();
        actual.copyTo(img_old);
        // ROS_INFO("ring_time: %.4f\n align_first_time:%.4f\n align_second_time:%.4f\n pub_vel_time:%.4f\n control_time:%.4f\n draw_time:%.4f\n", 
        //       ring_time, align_first_time, align_second_time, pub_vel_time, control_time, draw_time);
        contIMG++;
    }
    ROS_INFO("matching_result.mean_feature_error:%f", matching_result.mean_feature_error);
    if (matching_result.mean_feature_error < state.params.feature_threshold && contIMG > 3 && vel_msg.Vyaw < state.params.rotation_threshold)
    {
        cout << endl
             << "[INFO] Target reached within the feature threshold and maximum iterations" << endl
             << endl;
        target_reached_msg.data = true;
        target_reached_pub.publish(target_reached_msg);
        contIMG = 0;
    }
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
vector<Eigen::Vector3d> ibvsCircle::alignDepth2Color(Mat color, Mat depth, vector<Eigen::Vector3d> pd_uv, vc_parameters &params){
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
    Eigen::Isometry3d T_Depth2Color=Eigen::Isometry3d::Identity();
    T_Depth2Color.rotate(R_matrix);
    T_Depth2Color.pretranslate(t_vector);

	//获取深度像素与现实单位比例（D435默认1毫米）
    // float depth_scale = 1/1000.0;

	//初始化结果
	int num = pd_uv.size();
	vector<Eigen::Vector3d> pc_uv(num);

	Eigen::Vector3d pdd3;//彩色坐标系坐标
	//对深度图待对齐点遍历
	for(int i=0; i<num; i++){
		pdd3[0]=(pd_uv[i][0]-intrinDepth_matrix(0,2))/intrinDepth_matrix(0,0);
		pdd3[1]=(pd_uv[i][1]-intrinDepth_matrix(1,2))/intrinDepth_matrix(1,1);
		pdd3[2]=1;
		pc_uv[i] = intrinColor_matrix * T_Depth2Color * pdd3;
	}
    
    return pc_uv;
}

/*
	Function: getDepthImageFromMsg
	description: get the depth image in the format of cv_bridge::CvImagePtr from d435i
	params: message with depth image from d435i
*/
cv::Mat ibvsCircle::getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
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
// void ibvsCircle::poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
// {
// 	// cout << endl << "[INFO] poseCallback function" << endl;

// 	// Creating quaternion
// 	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
// 	// Creatring rotation matrix ffrom quaternion
// 	tf::Matrix3x3 mat(q);
// 	// obtaining euler angles
// 	double roll, pitch, yaw;
// 	mat.getEulerYPR(yaw, pitch, roll);
// 	// saving the data obtained
// 	state.Roll = (float)roll;
// 	state.Pitch = (float)pitch;
// 	state.Yaw = (float)yaw;
// 	// setting the position if its the first time
// 	if (!state.initialized)
// 	{
// 		cout << "[INFO] Setting initial position" << endl;
// 		state.initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
// 	}
// }

void ibvsCircle::quadCallback(const sensor_msgs::Imu::ConstPtr &msg){
	state.quad.w() = msg->orientation.w;
	state.quad.x() = msg->orientation.x;
	state.quad.y() = msg->orientation.y;
	state.quad.z() = msg->orientation.z;
	// cout << "quad:" << state.quad.w() << ", " << state.quad.x() << ", " << state.quad.y() << ", " << state.quad.z() << endl;
}

}