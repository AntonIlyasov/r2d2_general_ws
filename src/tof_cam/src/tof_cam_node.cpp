#pragma once
#include "ros/ros.h"
#include <iostream>
#include "tof_cam.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tof_cam_node");
  ros::NodeHandle n;

  Tof_cam my_cam;

  my_cam.printColorSensorInfo();

  while(1){

    my_cam.publishColorFrame();
    my_cam.publishDepthFrame16C1();
    my_cam.publishIrFrame();

    // my_cam.getColorFrame();
    // my_cam.getDepthFrame();
    // my_cam.getIrFrame();

    int framePerSecond = 30;
    char character = cv::waitKey(1000 / framePerSecond);
    if (character == 27) break;
  }

  return 0;
}