// #include "circle_detection/EDLib.h"
// #include <iostream>
// #include <opencv2/core/core.hpp> 
// #include <opencv2/highgui/highgui.hpp> 
// #include<opencv2/imgproc/imgproc.hpp>  
// using namespace cv;
// using namespace std;

// //bgr图像  
// Mat bgr;
// //HSV图像  
// Mat hsv;
// //色相  
// int hmin = 0;//h分量提取下限
// int hmax = 17;//h分量提取上限
// int h_Max = 180; //h分量可取的最大值
// //饱和度  
// int smin = 43;//s分量提取下限
// int smax = 255;//s分量提取上限
// int s_Max = 255;//s分量可取的最大值
// //亮度  
// int vmin = 0;//v分量提取下限
// int vmax = 255;//v分量提取上限
// int v_Max = 255;//v分量可取的最大值
// //显示原图的窗口  
// string windowName = "src";
// //输出图像的显示窗口  
// string dstName = "dst";
// //输出图像  
// Mat dst;
 

// int main()
// {
// 	// ---------------------orginal code ------------------------
// 	//输入图像  
// 	Mat srcImage = imread("/home/racheraven/Drone2/Drone824/src/circle_detection/src/frame0003.jpg");
// 	if (!srcImage.data) {
// 		cout << "falied to read" << endl;
// 		system("pause");
// 		return -1;
// 	}

// 	bgr = srcImage;
// 	medianBlur(bgr, bgr, 3);
// 	//颜色空间转换  
// 	cvtColor(bgr, hsv, COLOR_BGR2HSV);
// 	//输出图像分配内存  
// 	dst = Mat::zeros(bgr.size(), bgr.type());
// 	//遮罩  
// 	Mat mask;
// 	Mat lower_range;
// 	Mat upper_range;
// 	//inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
// 	inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_range);
// 	inRange(hsv, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_range);
// 	addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, mask);
// 	//只保留  
// 	for (int r = 0; r < bgr.rows; r++)
// 	{
// 		for (int c = 0; c < bgr.cols; c++)
// 		{
// 			if (mask.at<uchar>(r, c) == 255)
// 			{
// 				dst.at<Vec3b>(r, c)[0] = bgr.at<Vec3b>(r, c)[0];
// 				dst.at<Vec3b>(r, c)[1] = bgr.at<Vec3b>(r, c)[1];
// 				dst.at<Vec3b>(r, c)[2] = bgr.at<Vec3b>(r, c)[2];
// 			}
// 		}
// 	}

// 	imshow("mask", mask);
// 	//输出图像  
// 	imshow("dst", dst);

// 	waitKey();

// 	//---------------------end of orginal code ------------------------
// 	// Mat srcImage = imread("/home/racheraven/AirSim/1/18depth.png");
// 	// if (!srcImage.data) {
// 	// 	cout << "falied to read" << endl;
// 	// 	system("pause");
// 	// 	return -1;
// 	// }
// 	// dst = srcImage.clone();
// 	// for (int r = 0; r < srcImage.rows; r++){
// 	// 	for (int c = 0; c < srcImage.cols; c++){
// 	// 		if (srcImage.at<uchar>(r, c) > 13){
// 	// 			dst.at<uchar>(r, c) = 255;
// 	// 		}
// 	// 	}
// 	// }


// 	//***************************** ED Edge Segment Detection *****************************
// 	//Detection of edge segments from an input image	
// 	Mat testImg = dst;
// 	imshow("Source Image", testImg);

// 	//Call ED constructor
// 	ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm

// 	//Show resulting edge image
// 	Mat edgeImg = testED.getEdgeImage();
// 	//imshow("Edge Image - PRESS ANY KEY TO CONTINUE", edgeImg);
// 	waitKey();

// 	//Output number of segments
// 	int noSegments = testED.getSegmentNo();
// 	std::cout << "Number of edge segments: " << noSegments << std::endl;

// 	//Get edges in segment form (getSortedSegments() gives segments sorted w.r.t. legnths) 
// 	std::vector< std::vector<Point> > segments = testED.getSegments();


// 	//***************************** EDLINES Line Segment Detection *****************************
// 	//Detection of line segments from the same image
// 	EDLines testEDLines = EDLines(testImg);
// 	Mat lineImg = testEDLines.getLineImage();	//draws on an empty image
// 	//imshow("Line Image 1 - PRESS ANY KEY TO CONTINUE", lineImg);

// 	//Detection of lines segments from edge segments instead of input image
// 	//Therefore, redundant detection of edge segmens can be avoided
// 	testEDLines = EDLines(testED);
// 	lineImg = testEDLines.drawOnImage();	//draws on the input image
// 	//imshow("Line Image 2  - PRESS ANY KEY TO CONTINUE", lineImg);

// 	//Acquiring line information, i.e. start & end points
// 	vector<LS> lines = testEDLines.getLines();
// 	int noLines = testEDLines.getLinesNo();
// 	std::cout << "Number of line segments: " << noLines << std::endl;

// 	waitKey();

// 	//************************** EDPF Parameter-free Edge Segment Detection **************************
// 	// Detection of edge segments with parameter free ED (EDPF)

// 	EDPF testEDPF = EDPF(testImg);
// 	Mat edgePFImage = testEDPF.getEdgeImage();
// 	//imshow("Edge Image Parameter Free", edgePFImage);
// 	cout << "Number of edge segments found by EDPF: " << testEDPF.getSegmentNo() << endl;
// 	waitKey();

// 	//***************************** EDCIRCLES Circle Segment Detection *****************************
// 	//Detection of circles directly from the input image

// 	EDCircles testEDCircles = EDCircles(testImg);
// 	Mat circleImg = testEDCircles.drawResult(false, ImageStyle::CIRCLES);
// 	//imshow("Circle Image 1", circleImg);

// 	//Detection of circles from already available EDPF or ED image
// 	testEDCircles = EDCircles(testEDPF);

// 	//Get circle information as [cx, cy, r]
// 	vector<mCircle> circles = testEDCircles.getCircles();

// 	//Get ellipse information as [cx, cy, a, b, theta]
// 	vector<mEllipse> ellipses = testEDCircles.getEllipses();

// 	//Circles and ellipses will be indicated in green and red, resp.
// 	circleImg = testEDCircles.drawResult(true, ImageStyle::BOTH);
// 	imshow("CIRCLES and ELLIPSES RESULT IMAGE", circleImg);

// 	int noCircles = testEDCircles.getCirclesNo();
// 	std::cout << "Number of circles: " << noCircles << std::endl;
// 	waitKey();

// 	//*********************** EDCOLOR Edge Segment Detection from Color Images **********************

// 	Mat colorImg = dst;
// 	//Mat colorImg = imread("billiardNoise.jpg");
// 	EDColor testEDColor = EDColor(colorImg, 36, 4, 1.5, true); //last parameter for validation
// 	//imshow("Color Edge Image - PRESS ANY KEY TO QUIT", testEDColor.getEdgeImage());
// 	cout << "Number of edge segments detected by EDColor: " << testEDColor.getSegmentNo() << endl;
// 	waitKey();

// 	// get lines from color image
// 	EDLines colorLine = EDLines(testEDColor);
// 	//imshow("Color Line", colorLine.getLineImage());
// 	std::cout << "Number of line segments: " << colorLine.getLinesNo() << std::endl;
// 	waitKey();

// 	// get circles from color image
// 	EDCircles colorCircle = EDCircles(testEDColor);
// 	// TO DO :: drawResult doesnt overlay (onImage = true) when input is from EDColor
// 	circleImg = colorCircle.drawResult(false, ImageStyle::BOTH);
// 	imshow("Color Circle", circleImg);
// 	std::cout << "Number of line segments: " << colorCircle.getCirclesNo() << std::endl;
// 	waitKey();

// 	return 0;
// }



