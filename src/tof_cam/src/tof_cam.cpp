#pragma once
#include "tof_cam.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Tof_cam::Tof_cam(){
  openni::OpenNI::initialize();
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed");
  }
  printf("oni.init()\n");
  
  pubColor  = nh.advertise<sensor_msgs::Image>(TO_COLOR_TOPIC, 1);
  pubDepth  = nh.advertise<sensor_msgs::Image>(TO_DEPTH_TOPIC, 1);
  pubIr     = nh.advertise<sensor_msgs::Image>(TO_IR_TOPIC, 1);

  commands_sub = nh.subscribe(FROM_COMMAND_TOPIC, 1, &Tof_cam::fromCommandTopicCallback, this);
  commandFromTopic = 0;
}

Tof_cam::~Tof_cam(){
  openni::OpenNI::shutdown();
}

Tof_cam::Tof_cam(int a){}

void Tof_cam::fromCommandTopicCallback(const std_msgs::Int32& msg) {
  std::cout << "\033[1;34m fromCommandTopicCallback \033[0m\n";
  commandFromTopic = msg.data;
}

int Tof_cam::getCommandFromTopic(){
  return commandFromTopic;
}

void Tof_cam::getColorFrame(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrame;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1280_720_RGB888_30FPS);
  std::cout << "getColorFrame:            " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrame);
  std::cout << colorFrame.size() << "\n";
  if(!colorFrame.empty()) cv::imshow("Color", colorFrame);
}

void Tof_cam::getColorFrameMaxQuality(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrameMaxQuality;  
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1920_1080_RGB888_15FPS);
  std::cout << "getColorFrameMaxQuality:  " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);
  std::cout << colorFrameMaxQuality.size() << "\n";
  if(!colorFrameMaxQuality.empty()) cv::imshow("colorFrameMaxQuality", colorFrameMaxQuality);
}

void Tof_cam::getDepthFrame(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame;
  oni.getDepthFrame(depthFrame);
  if(!depthFrame.empty()) cv::imshow("Depth", depthFrame);
}

void Tof_cam::getIrFrame(){
  if (!oni.m_irStreamIsValid()) return;
  cv::Mat irFrame;
  oni.getIrFrame(irFrame);
  if(!irFrame.empty())    cv::imshow("IR",    irFrame);
}

void Tof_cam::printColorSensorInfo(){
  oni.printColorSensorInfo();
}

void Tof_cam::publishColorFrame(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrame;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1280_720_RGB888_30FPS);
  std::cout << "publishColorFrame: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrame);

  if(!colorFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = colorFrame;
    pubColor.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::publishColorFrameMaxQuality(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrameMaxQuality;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1920_1080_RGB888_15FPS);
  std::cout << "publishColorFrameMaxQuality: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);

  if(!colorFrameMaxQuality.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = colorFrameMaxQuality;
    pubColor.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::publishDepthFrame16C1(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame16C1;
  // std::cout << "publishDepthFrame16C1         RESOLUTION: " << oni.getDepthResolutionX() << "x" << oni.getDepthResolutionY() << "\n";
  oni.getDepthFrame16C1(depthFrame16C1);

  if(!depthFrame16C1.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_msg.image = depthFrame16C1;
    pubDepth.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::publishIrFrame(){
  if (!oni.m_irStreamIsValid()) return;
  cv::Mat irFrame;
  // std::cout << "publishIrFrame                RESOLUTION: " << oni.getIrResolutionX() << "x" << oni.getIrResolutionY() << "\n";
  oni.getIrFrame(irFrame);

  if(!irFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = irFrame;
    pubIr.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::shutdownTofCam(){
  openni::OpenNI::shutdown();
  oni.~OpenNI2OpenCV();
}

void Tof_cam::resetTofCam(){
  openni::OpenNI::shutdown();
  oni.~OpenNI2OpenCV();
  openni::OpenNI::initialize();
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed");
  }
  printf("oni.init()\n");
}

void Tof_cam::turnOnTofCam(){
  openni::OpenNI::initialize();
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed");
  }
  printf("oni.init()\n");
}