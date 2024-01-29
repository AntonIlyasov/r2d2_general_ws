#pragma once
#include "ros/ros.h"
#include <iostream>
#include "tof_cam.h"
#include <opencv2/core/ocl.hpp>

using namespace std;

int main(int argc, char **argv)
{

  if (!cv::ocl::haveOpenCL())
  {
    cout << "OpenCL is not available..." << endl;
    return -1;
  } else {
    cout << "OpenCL is available..." << endl;
  }

  if (cv::ocl::haveOpenCL())
  {
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
      std::cerr << "Failed creating the OpenCL context" << std::endl;
      return -1;
    }
    cv::ocl::setUseOpenCL(true);
  }

  ros::init(argc, argv, "tof_cam_node");
  Tof_cam my_cam;

  while(ros::ok()){

    ros::spinOnce();

    // my_cam.getIrFrame();
    // my_cam.getDepthFrame();
    // my_cam.getDepthFrame16C1();

    my_cam.publishDepthFrame16C1();
    my_cam.publishIrFrame();
    
    switch (my_cam.getCommandFromTopic())
    {
    case 0:
      my_cam.shutdownTofCam();
      break;
    case 1:
      my_cam.resetTofCam();
      break;
    case 2:
      my_cam.turnOnTofCam();
      break;
    case 3:
      my_cam.publishColorFrame();
      break;
    case 4:
      my_cam.publishColorFrameMaxQuality();
      break;
    default:
      break;
    }

    int framePerSecond = 30;
    char character = cv::waitKey(1000 / framePerSecond);
    if (character == 27) break;
  }

  return 0;
}