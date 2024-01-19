#pragma once
#include "OpenNI2OpenCV.h"
#include <ros/ros.h>

#define TO_COLOR_TOPIC      "toColorTopic"
#define TO_DEPTH_TOPIC      "toDepthTopic"
#define TO_IR_TOPIC         "toIrTopic"

class Tof_cam{
public:
  // constructors & destructors
  Tof_cam();
  ~Tof_cam();
  Tof_cam(int a);
  
  // methods
  void getColorFrame();
  void getColorFrameMaxQuality();
  void getDepthFrame();
  void getIrFrame();

  void publishColorFrame();
  void publishColorFrameMaxQuality();
  void publishDepthFrame16C1();
  void publishIrFrame();

  void printColorSensorInfo();

private:
  OpenNIOpenCV::OpenNI2OpenCV oni;
  ros::NodeHandle nh;
  ros::Publisher pubColor;
  ros::Publisher pubDepth;
  ros::Publisher pubIr;
};