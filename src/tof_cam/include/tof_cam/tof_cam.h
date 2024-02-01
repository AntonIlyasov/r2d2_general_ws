#pragma once
#include "OpenNI2OpenCV.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <boost/asio.hpp>

#define TO_COLOR_TOPIC      "toColorTopic"
#define TO_DEPTH_TOPIC      "toDepthTopic"
#define TO_IR_TOPIC         "toIrTopic"
#define FROM_COMMAND_TOPIC  "fromCommandTopic"

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
  void getDepthFrame16C1();
  void getIrFrame();

  void publishColorFrame();
  void publishColorFrameMaxQuality();
  void saveColorFrameMaxQuality();
  void sendToTCPColorFrameMaxQuality();
  void publishDepthFrame16C1();
  void publishIrFrame();

  void printColorSensorInfo();

  int getCommandFromTopic();
  void shutdownTofCam();
  void resetTofCam();
  void turnOnTofCam();

private:
  OpenNIOpenCV::OpenNI2OpenCV oni;
  ros::NodeHandle nh;
  ros::Publisher pubColor;
  ros::Publisher pubDepth;
  ros::Publisher pubIr;
  ros::Subscriber commands_sub;

  int commandFromTopic;

  void fromCommandTopicCallback(const std_msgs::Int32&);

  int countBmp;
};