#pragma once
#include "ros/ros.h"
#include <iostream>
#include "tof_cam.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tof_cam_node");
  Tof_cam my_cam;

  while(ros::ok()){

    my_cam.publishDepthFrame16C1();
    my_cam.publishIrFrame();

    switch (my_cam.getCommandFromTopic())
    {
    case 0:
      my_cam.publishColorFrame();
      break;
    case 1:
      my_cam.publishColorFrameMaxQuality();
      break;
    case 2:
      my_cam.shutdownTofCam();
      break;
    case 3:
      my_cam.resetTofCam();
      break;
    default:
      break;
    }

    int framePerSecond = 30;
    char character = cv::waitKey(1000 / framePerSecond);
    if (character == 27) break;

    ros::spinOnce();
  }

  return 0;
}