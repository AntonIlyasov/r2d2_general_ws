// #include "ros/ros.h"
// #include <iostream>
// #include "OpenNI2OpenCV.h"

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "tof_cam_node");
//   ros::NodeHandle n;

//   OpenNIOpenCV::OpenNI2OpenCV oni;
//   if (oni.init() != openni::STATUS_OK){
//     printf("Initializatuion failed");
//     return 1;
//   }
//   printf("oni.init()\n");

//   cv::Mat colorFrame, depthFrame, irFrame;
//   oni.setColorVideoMode(6);


//   while (ros::ok()){
//     oni.getColorFrame(colorFrame);
//     oni.getDepthFrame16C1(depthFrame);
//     // oni.getIrFrame(irFrame);
//     if(!colorFrame.empty()) cv::imshow("Color", colorFrame);
//     if(!depthFrame.empty()) cv::imshow("Depth", depthFrame);
//     // if(!irFrame.empty())    cv::imshow("IR",    irFrame);
    
//     int framePerSecond = 30;
//     char character = cv::waitKey(1000 / framePerSecond);
//     if (character == 27) break;
//   }
  
//   openni::OpenNI::shutdown();
//   return 0;
// }

/////////////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace cv::ocl;
using namespace std;


int main(int argc, char **argv)
{
  
  if (!cv::ocl::haveOpenCL())
  {
    cout << "OpenCL is not available..." << endl;
    return 0;
  }

  ros::init(argc, argv, "tof_cam_node");

//     TickMeter tm;
//     tm.start();
// if (cv::ocl::haveOpenCL())
//     {
//         cv::ocl::Context context;
//         if (!context.create(cv::ocl::Device::TYPE_GPU))
//         {
//             std::cerr << "Failed creating the OpenCL context" << std::endl;
//             return -1;
//         }
//        cv::ocl::setUseOpenCL(true);
//    }
//     cv::UMat output;
//     cv::UMat image1 = cv::imread("test1.png").getUMat(cv::ACCESS_RW);
//     cv::UMat image2 = cv::imread("test2.png").getUMat(cv::ACCESS_RW);
//     mergeImage(image1, image2, output);
//     imwrite("output.png", output);
//     tm.stop();
// 	std::cout<<"count="<<tm.getCounter()<<" ,process time="<<tm.getTimeMilli()<<" ms."<<std::endl;
 
	return 0;
}