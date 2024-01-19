#pragma once
#include "tof_cam.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Tof_cam::Tof_cam(){
  
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed");
  }
  printf("oni.init()");

  pubColor  = nh.advertise<sensor_msgs::Image>(TO_COLOR_TOPIC, 1);
  pubDepth  = nh.advertise<sensor_msgs::Image>(TO_DEPTH_TOPIC, 1);
  pubIr     = nh.advertise<sensor_msgs::Image>(TO_IR_TOPIC, 1);
}

Tof_cam::~Tof_cam(){
  openni::OpenNI::shutdown();
}

Tof_cam::Tof_cam(int a){}

void Tof_cam::getColorFrame(){
  cv::Mat colorFrame;
  oni.setColorResolution(1280, 720);
  oni.setColorFps(30);
  oni.setColorPixelFormat(PIXEL_FORMAT_RGB888);

  std::cout << "getColorFrame: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrame);
  if(!colorFrame.empty()) cv::imshow("Color", colorFrame);
}

void Tof_cam::getColorFrameMaxQuality(){
  cv::Mat colorFrameMaxQuality;
  oni.setColorResolution(1920, 1080);
  oni.setColorFps(15);
  oni.setColorPixelFormat(PIXEL_FORMAT_RGB888);

  std::cout << "getColorFrameMaxQuality: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);
  if(!colorFrameMaxQuality.empty()) cv::imshow("colorFrameMaxQuality", colorFrameMaxQuality);
}

void Tof_cam::getDepthFrame(){
  cv::Mat depthFrame;
  oni.getDepthFrame(depthFrame);
  if(!depthFrame.empty()) cv::imshow("Depth", depthFrame);
}

void Tof_cam::getIrFrame(){
  cv::Mat irFrame;
  oni.getIrFrame(irFrame);
  if(!irFrame.empty())    cv::imshow("IR",    irFrame);
}

void Tof_cam::printColorSensorInfo(){
  oni.printColorSensorInfo();
}

void Tof_cam::publishColorFrame(){
  cv::Mat colorFrame;
  oni.setColorResolution(1280, 720);
  oni.setColorFps(30);
  oni.setColorPixelFormat(PIXEL_FORMAT_RGB888);

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
  cv::Mat colorFrameMaxQuality;
  oni.setColorResolution(1920, 1080);
  oni.setColorFps(15);
  oni.setColorPixelFormat(PIXEL_FORMAT_RGB888);

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
  cv::Mat depthFrame16C1;
  std::cout << "publishDepthFrame16C1         RESOLUTION: " << oni.getDepthResolutionX() << "x" << oni.getDepthResolutionY() << "\n";
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
  cv::Mat irFrame;
  std::cout << "publishIrFrame                RESOLUTION: " << oni.getIrResolutionX() << "x" << oni.getIrResolutionY() << "\n";
  oni.getIrFrame(irFrame);

  if(!irFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = irFrame;
    pubIr.publish(out_msg.toImageMsg());
  }

}