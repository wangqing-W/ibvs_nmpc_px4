#include "circle_detection/CircleDetection.h"
#define Display 0
#define Display_forDebug 0
#define REALRACE 0

#define TWOCIRCLES 2
#define ONECIRCLESONEELLIPSE 1
#define TWOELLIPSE 0

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define farawayTH 3000

using namespace cv;
using namespace std;
using namespace Eigen;

namespace circle_detection{
CircleDetection::CircleDetection(ros::NodeHandle nh):n(nh){
        init();
}
CircleDetection::~CircleDetection(){};

#if REALRACE
void CircleDetection::init(){
    ros::param::get("~d435i_depth_topic", depth_topic);
    camera_K_<< 695.9951171875, 0, 640.0, 0, 695.9951171875, 360.0, 0, 0, 1;
    camera_K_inv_ = camera_K_.inverse();
    
    ROS_INFO_STREAM("camera parameters\n" << camera_K_inv_);
    ROS_INFO("Circle Detection initialized!!");

    sub_depthimg = n.subscribe(depth_topic, 1, &CircleDetection::imgdepth_callback, this, ros::TransportHints().tcpNoDelay());
    pub_recg = n.advertise<custom_msgs::CircleRecog>("/D435i/depth_recogization", 1);
}
#else
void CircleDetection::init(){
    camera_K_<< 347.99755859375, 0, 320.0, 0, 347.99755859375, 240.0, 0, 0, 1;
    camera_K_inv_ = camera_K_.inverse();
    
    ROS_INFO_STREAM("camera parameters\n" << camera_K_inv_);
    ROS_INFO("Circle Detection initialized!!");

    pub_recg = n.advertise<custom_msgs::CircleRecog>("/d435/depth_recogization", 1);
}
#endif


void CircleDetection::detectRing(cv::Mat depth, bool desired){
    ros::Time t0, t1;
    double get_time, remove_time, int_time, equ_time, detect_time, draw_time, circle_time;
    // t0 = ros::Time::now();
    // depth_buf = getDepthImageFromMsg(img_msg);
    depth_buf = depth;
    // get_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();

    // ROS_INFO("get image");
    cv::Mat img_before, img;
    // if (depth_buf.empty()){printf("No Depth Image"); return;}
    RemoveFarAwayPoints(depth_buf); //remove points farther away than 5m
    remove_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();
    img_before = CircleFloat2Int(depth_buf);     //convert from float to int
    int_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();

    img = EqualizeImg(img_before);     //convert from float to int
    equ_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();
    // ROS_INFO("before draw");

    EDCircles testEDCircles = EDCircles(img); //detect circles and ellipses
    detect_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();

    int noCircles = testEDCircles.getCirclesNo();
	// std::cout << "Number of circles: " << noCircles << std::endl;
    // ROS_INFO("circleDetected");
    cv::Mat circleImg = testEDCircles.drawResult(true, ImageStyle::BOTH);
    if (Display) {
        if(!desired){
            // cout << depth_buf.at<uint16_t>(depth_buf.rows/2, depth_buf.cols/2);
            cv::namedWindow("DepthImage", cv::WINDOW_NORMAL);
            cv::moveWindow("DepthImage", 0, 0);
            cv::resizeWindow("DepthImage", 640, 480);
            cv::imshow("DepthImage", img_before);
            // cv::imwrite("/home/zyh/ibvs_rpg_ws/src/ibvs_pkg/target/depthimg.png", img);
            cv::waitKey(1);

            cv::namedWindow("DepthImageAfterEqualize", cv::WINDOW_NORMAL);
            cv::moveWindow("DepthImageAfterEqualize", 0, 480);
            cv::resizeWindow("DepthImageAfterEqualize", 640, 480);
            cv::imshow("DepthImageAfterEqualize", circleImg);
            cv::waitKey(1);
        }
    }
    draw_time = (ros::Time::now()-t0).toSec();
    t0 = ros::Time::now();
    // ROS_INFO("draw");


    

    if (testEDCircles.getCirclesNo() == 0 && testEDCircles.getEllipsesNo() ==0 ){
        ROS_INFO("No circles");
        return;
    }
    if ((testEDCircles.getCirclesNo() + testEDCircles.getEllipsesNo()) == 2){
        vector<mCircle> circles = testEDCircles.getCircles();
        vector<mEllipse> ellipses = testEDCircles.getEllipses();
        Eigen::Vector3d CenterPt;
        switch(testEDCircles.getCirclesNo()){
            case TWOCIRCLES:
                return getDepth2Circles(circles);
            break;
            case ONECIRCLESONEELLIPSE:
                return getDepth1Circle1Ellipse(circles, ellipses);
            break;
            case TWOELLIPSE:
                return getDepth2Ellipses(ellipses);
            break;
        }
    }
    circle_time = (ros::Time::now()-t0).toSec();
    // ROS_INFO("Get image time: %.4f\n Remove time:%.4f\n toInt time:%.4f\n Equ time:%.4f\n Detect time:%.4f\n Draw time:%.4f\n Circle time:%.4f\n ", 
    //           get_time, remove_time, int_time, equ_time, detect_time, draw_time, circle_time);

}
//converting from topic to depth image
#if REALRACE

cv::Mat CircleDetection::getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    //depth has encoding TYPE_16UC1
    cv_bridge::CvImageConstPtr depth_ptr;
    depth_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat img = depth_ptr->image.clone();
    return img;
}
#else

// cv::Mat CircleDetection::getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     //depth has encoding TYPE_32FC1
//     cv_bridge::CvImageConstPtr depth_ptr;
//     depth_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
//     cv::Mat img = depth_ptr->image.clone();
//     cv::Mat img_afterTH(cv::Size(img.cols, img.rows),CV_16UC1);
//     // cout << img_afterTH.rows << " " << img_afterTH.cols << endl;
//         // depth has encoding TYPE_16UC1
//     for (int row = 0; row < img.rows; row++){
//         for (int col = 0; col < img.cols; col++){    
//             /* 注意 Mat::at 函数是个模板函数, 需要指明参数类型, 因为这张图是具有红蓝绿三通道的图,
//                所以它的参数类型可以传递一个 Vec3b, 这是一个存放 3 个 uchar 数据的 Vec(向量). 这里
//                提供了索引重载, [2]表示的是返回第三个通道, 在这里是 Red 通道, 第一个通道(Blue)用[0]返回 */
//                 img_afterTH.at<uint16_t>(row, col) = (uint16_t)(img.at<float>(row, col) * 1000.0);
//         }
//     }
//     return img_afterTH;
// }
cv::Mat CircleDetection::getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
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
#endif



//converting from float to gray, throw away points farther away than 5m
void CircleDetection::RemoveFarAwayPoints(cv::Mat src)
{
    // depth has encoding TYPE_16UC1
    for (int row = 0; row < src.rows; row++){
        for (int col = 0; col < src.cols; col++){    
            /* 注意 Mat::at 函数是个模板函数, 需要指明参数类型, 因为这张图是具有红蓝绿三通道的图,
               所以它的参数类型可以传递一个 Vec3b, 这是一个存放 3 个 uchar 数据的 Vec(向量). 这里
               提供了索引重载, [2]表示的是返回第三个通道, 在这里是 Red 通道, 第一个通道(Blue)用[0]返回 */
            if(src.at<uint16_t>(row, col) > uint16_t(farawayTH))
                src.at<uint16_t>(row, col) = uint16_t(0);
        }
    }
}

cv::Mat CircleDetection::CircleFloat2Int(cv::Mat src){
    cv::Mat img;
    img = cv::Mat::zeros(src.size(), CV_8UC1);

    for (int row=0; row<src.rows; ++row){
        for (int col=0; col<src.cols; col++){
            img.at<uchar>(row, col) = src.at<uint16_t>(row, col) >> 8;
            // pixel_distance = 0.001*(cv_ptr->image.at<u_int16_t>(pixel_height,pixel_width));
        }
    }
    return img; 
}

cv::Mat CircleDetection::EqualizeImg(cv::Mat src){
    cv::Mat dst;
    equalizeHist(src, dst);
    return dst;
}


void CircleDetection::getDepth2Circles(const std::vector<mCircle> &circles){
    ROS_INFO("Detect 2 circles"); 
    cv::Point2d center0 = circles[0].center;
    cv::Point2d center1 = circles[1].center;
    
    double rmax = circles[0].r >= circles[1].r ? circles[0].r : circles[1].r;
    double rmin = circles[0].r < circles[1].r ? circles[0].r : circles[1].r;
    int r = 0;
    if (abs(rmax - rmin) < hypot((center0.x - center1.x), (center0.y - center1.y))) {
        ROS_INFO("Crossing circles, not ring, skip"); 
        return;
    }
    if (rmax > 2*rmin) {
        ROS_WARN("Abnormal ring size, skip");
        return;
    }

    cv::Point2d RingCenter((center0.x + center1.x)/2, (center0.y + center1.y)/2);
    if (RingCenter.x < 0 || RingCenter.x > IMAGE_WIDTH || RingCenter.y < 0 || RingCenter.y > IMAGE_HEIGHT) {
        ROS_INFO("Half of the circle outside, skip");
        return;
    }
    

    mCircle CircleOuter(RingCenter, rmax);
    mCircle CircleInside(RingCenter, rmin);
    // std::vector<Eigen::Vector3d> candidateCameraPt;
    float Centerdepth = 0;

    cv::Mat dst;
    dst = cv::Mat::zeros(depth_buf.size(), CV_8UC1);
    int idx = 0;

    for (int row=0; row<depth_buf.rows; ++row){
        for (int col=0; col<depth_buf.cols; col++){
           if (InsideCircle(CircleOuter, col, row) && OutsideCircle(CircleInside, col, row)){
               if (Display_forDebug){
                    dst.at<uint8_t>(row, col) = 255;
               }
                Eigen::Vector3d ImagePt(col, row, depth_buf.at<uint16_t>(row, col));
                Eigen::Vector3d CameraPt = FromImage2CameraFrame(ImagePt);
                // candidateCameraPt.push_back(CameraPt);
                if (CameraPt(2) >= 1000 || CameraPt(2) <= 2500){
                    Centerdepth = Centerdepth + CameraPt(2);
                    ++idx;
                }
           }
        }
    }
    if (idx <= 0){
        ROS_INFO("Not enough points");
        return;
    }
    Centerdepth = Centerdepth / float(idx);
    Eigen::Vector3d CenterPt = FromImage2CameraFrame(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth));
    ROS_INFO_STREAM("current ring center:" << RingCenter.x  << ' ' << RingCenter.y << ' ' << Centerdepth);
    ROS_INFO_STREAM("current ring center in real world is:" << CenterPt(0) << ' ' << CenterPt(1)<< ' ' << CenterPt(2));
    if (Display_forDebug) {
        cv::namedWindow("Circle", cv::WINDOW_NORMAL);
        cv::moveWindow("Circle", 1920, 0);
        cv::resizeWindow("Circle", 640, 480);
        cv::imshow("Circle", dst);
        cv::waitKey(1); 
    }
    PublishResult(CenterPt);
    ring_result = make_pair(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth), Eigen::Vector2d((rmax+rmin)/2, (rmax+rmin)/2));
}

//-------------------if 1 circle and 1 ellipse-------------
void CircleDetection::getDepth1Circle1Ellipse(const std::vector<mCircle> &circles, const std::vector<mEllipse> &ellipses){
    ROS_INFO("Detect 1 circle and 1 ellipse"); 
    cv::Point2d center0 = circles[0].center;
    cv::Point2d center1 = ellipses[0].center;

    int r = 0;
    if (abs(circles[0].r - ellipses[0].axes.width) < hypot((center0.x - center1.x), (center0.y - center1.y)) &&  abs(circles[0].r -ellipses[0].axes.height) < hypot((center0.x - center1.x), (center0.y - center1.y))) {
        ROS_INFO("Crossing circle and ellipse, not ring, skip");
        return;
    }

    cv::Point2d RingCenter((center0.x + center1.x)/2, (center0.y + center1.y)/2);
    if (RingCenter.x < 0 || RingCenter.x > IMAGE_WIDTH || RingCenter.y < 0 || RingCenter.y > IMAGE_HEIGHT) {
        ROS_INFO("Half of the circle outside, skip");
        return;
    }
    
    // std::vector<Eigen::Vector3d> candidateCameraPt;
    float Centerdepth = 0;

    cv::Mat dst;
    dst = cv::Mat::zeros(depth_buf.size(), CV_8UC1);
    int idx = 0;

    for (int row=0; row<depth_buf.rows; ++row){
        for (int col=0; col<depth_buf.cols; col++){
            if((InsideEllipse(ellipses[0], col, row) && !InsideCircle(circles[0], col, row)) || (InsideCircle(circles[0], col, row) && !InsideEllipse(ellipses[0], col, row))){
               if (Display_forDebug){
                    dst.at<uint8_t>(row, col) = 255;
               }
                Eigen::Vector3d ImagePt(col, row, depth_buf.at<uint16_t>(row, col));
                Eigen::Vector3d CameraPt = FromImage2CameraFrame(ImagePt);
                // candidateCameraPt.push_back(CameraPt);
                if (CameraPt(2) >= 1000 || CameraPt(2) <= 2500){
                    Centerdepth = Centerdepth + CameraPt(2);
                    ++idx;
                }
           }
        }
    }
    if (idx <= 0){
        ROS_INFO("Not enough points");
        return;
    }
    Centerdepth = Centerdepth / float(idx);
    Eigen::Vector3d CenterPt = FromImage2CameraFrame(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth));
    ROS_INFO_STREAM("current ring center:" << RingCenter.x  << ' ' << RingCenter.y << ' ' << Centerdepth);
    ROS_INFO_STREAM("current image center in image is:" << CenterPt(0) << ' ' << CenterPt(1)<< ' ' << CenterPt(2));
    if (Display_forDebug) {
        cv::namedWindow("1e1c", cv::WINDOW_NORMAL);
        cv::moveWindow("1e1c", 1920, 0);
        cv::resizeWindow("1e1c", 640, 480);
        cv::imshow("1e1c", dst);
        cv::waitKey(1); 
    }
    PublishResult(CenterPt);
    ring_result = make_pair(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth), 
                            Eigen::Vector2d((circles[0].r+ellipses[0].axes.width)/2, (circles[0].r+ellipses[0].axes.height)/2));
}

//-------------------if 2 ellipses----------------
void CircleDetection::getDepth2Ellipses(const std::vector<mEllipse> &ellipses){
    ROS_INFO("Detect 2 ellipses"); 
    cv::Point2d center0 = ellipses[0].center;
    cv::Point2d center1 = ellipses[1].center;

    int r = 0;
    //may cause problem
    if (abs(ellipses[1].axes.height - ellipses[0].axes.height) < hypot((center0.x - center1.x), (center0.y - center1.y))) {
        ROS_INFO("Crossing circle and ellipse, not ring, skip");
        return;
    }

    cv::Point2d RingCenter((center0.x + center1.x)/2, (center0.y + center1.y)/2);
    if (RingCenter.x < 0 || RingCenter.x > IMAGE_WIDTH || RingCenter.y < 0 || RingCenter.y > IMAGE_HEIGHT) {
        ROS_INFO("Half of the ellipse outside, skip");
        return;
    }
    
    // std::vector<Eigen::Vector3d> candidateCameraPt;
    float Centerdepth = 0;

    cv::Mat dst;
    dst = cv::Mat::zeros(depth_buf.size(), CV_8UC1);
    int idx = 0;

    for (int row=0; row<depth_buf.rows; ++row){
        for (int col=0; col<depth_buf.cols; col++){
            if((InsideEllipse(ellipses[0], col, row) && !InsideEllipse(ellipses[1], col, row)) || (InsideEllipse(ellipses[0], col, row) && !InsideEllipse(ellipses[1], col, row))){
               if (Display_forDebug){
                    dst.at<uint8_t>(row, col) = 255;
               }
                Eigen::Vector3d ImagePt(col, row, depth_buf.at<uint16_t>(row, col));
                Eigen::Vector3d CameraPt = FromImage2CameraFrame(ImagePt);
                // candidateCameraPt.push_back(CameraPt);
                if (CameraPt(2) >= 1000 || CameraPt(2) <= 2500){
                    Centerdepth = Centerdepth + CameraPt(2);
                    ++idx;
                }
           }
        }
    }
    if (idx <= 0){
        ROS_INFO("Not enough points");
        return;
    }
    Centerdepth = Centerdepth / float(idx);
    Eigen::Vector3d CenterPt = FromImage2CameraFrame(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth));
    ROS_INFO_STREAM("current ring center:" << RingCenter.x  << ' ' << RingCenter.y << ' ' << Centerdepth);
    ROS_INFO_STREAM("current image center in image is:" << CenterPt(0) << ' ' << CenterPt(1)<< ' ' << CenterPt(2));


    if (Display_forDebug) {
        cv::namedWindow("2e", cv::WINDOW_NORMAL);
        cv::moveWindow("2e", 1920, 0);
        cv::resizeWindow("2e", 640, 480);
        cv::imshow("2e", dst);
        cv::waitKey(1); 
    }
    PublishResult(CenterPt);
    ring_result = make_pair(Eigen::Vector3d(RingCenter.x, RingCenter.y, Centerdepth), 
                            Eigen::Vector2d((ellipses[0].axes.width+ellipses[1].axes.width)/2, (ellipses[0].axes.height+ellipses[1].axes.height)/2));
}


bool CircleDetection::InsideCircle(const mCircle &circle, float ptx, float pty){
    // cout << circle.center.x << ' ' << circle.center.y<< ' ' << circle.r<< ' ' << pt[0] << ' '<< pt[1];
    return (ptx-circle.center.x)*(ptx-circle.center.x) + (pty-circle.center.y) * (pty-circle.center.y) <= circle.r * circle.r;
}

bool CircleDetection::OutsideCircle(const mCircle &circle, float ptx, float pty){
    // cout << circle.center.x << ' ' << circle.center.y<< ' ' << circle.r<< ' ' << pt[0] << ' '<< pt[1];
    return (ptx-circle.center.x)*(ptx-circle.center.x) + (pty-circle.center.y) * (pty-circle.center.y) >= circle.r * circle.r;
}

bool CircleDetection::InsideEllipse(const mEllipse &ellipse, float pt_x, float pt_y){
    // cout << ellipse.center.x << ' ' << ellipse.center.y<< ' ' << ellipse.theta<< ' ' << ellipse.axes.height << ' '<< ellipse.axes.width;
    return ((pt_x-ellipse.center.x)*cos(ellipse.theta)+(pt_y-ellipse.center.y)*sin(ellipse.theta))*((pt_x-ellipse.center.x)*cos(ellipse.theta)+(pt_y-ellipse.center.y)*sin(ellipse.theta))/(double)(ellipse.axes.width * ellipse.axes.width)
        + ((pt_y-ellipse.center.y)*cos(ellipse.theta)-(pt_x-ellipse.center.x)*sin(ellipse.theta))*((pt_y-ellipse.center.y)*cos(ellipse.theta)-(pt_x-ellipse.center.x)*sin(ellipse.theta))/(double)(ellipse.axes.height * ellipse.axes.height)
        <= 1.0;
}


Eigen::Vector3d CircleDetection::FromImage2CameraFrame(const Eigen::Vector3d &v1){
    Vector3d v(v1(0), v1(1), 1);
    return v1(2) * camera_K_inv_ * v;
}

void CircleDetection::PublishResult(const Eigen::Vector3d &CenterPt){
    custom_msgs::CircleRecog recg;
    recg.header.stamp = ros::Time::now();
    recg.header.frame_id = "body";

    //turn from camera frame to body frame, under camera frame, the unit is mm, under body the unit is m
    recg.point.x = double(CenterPt(2)/1000.0);
    recg.point.y = double(-1.0*CenterPt(0)/1000.0); 
    recg.point.z = double(-1.0*CenterPt(1)/1000.0) - 0.070;

    recg.vector.x = -1;
    recg.vector.y = 0;
    recg.vector.z = 0;


    pub_recg.publish(recg);
}

}
