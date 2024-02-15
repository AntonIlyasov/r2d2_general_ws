#pragma once
#include "tof_cam.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Tof_cam::Tof_cam(){
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed\n");
    exit(1);
  }
  printf("oni.init()\n");

  toTofCamControlPub   = nh.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_CONTROL_TOPIC_NAME, 0);
  fromTofCamControlSub = nh.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_CONTROL_TOPIC_NAME, 0, 
      &Tof_cam::fromTofCamControlCallback, this);

  pubColor  = nh.advertise<sensor_msgs::Image>(TO_COLOR_TOPIC_NAME, 0);
  pubDepth  = nh.advertise<sensor_msgs::Image>(TO_DEPTH_TOPIC_NAME, 0);
  pubIr     = nh.advertise<sensor_msgs::Image>(TO_IR_TOPIC_NAME,    0);

  memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
  memset(dataFromTofCamControl, 0, sizeof(dataFromTofCamControl));
}

// основной цикл программы
void Tof_cam::nodeProcess(){
  if (getMsgFromTofCamControl){
    getMsgFromTofCamControl = false;
    switch (getCmdFromTofCamControl())
    {
      case static_cast<int>(TofCamCmd::shutdown):             //0x00
        shutdownTofCam();
        break;
      case static_cast<int>(TofCamCmd::reset):                //0x01
        resetTofCam();
        break;
      case static_cast<int>(TofCamCmd::turnOn):               //0x02
        turnOnTofCam();
        break;
      case static_cast<int>(TofCamCmd::publishNormalQuality): //0x03
        publishColorFrame();
        publishDepthFrame16C1();
        publishIrFrame();
        break;
      case static_cast<int>(TofCamCmd::publishMaxQuality):    //0x04
        publishColorFrameMaxQuality();
        publishDepthFrame16C1();
        publishIrFrame();
        break;
      case static_cast<int>(TofCamCmd::saveMaxQuality):       //0x05
        publishColorFrameMaxQuality();
        publishDepthFrame16C1();
        publishIrFrame();
        break;
      default:
        break;
    }
    sendMsgToTofCamControl();
  }
}

void Tof_cam::fromTofCamControlCallback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
  getMsgFromTofCamControl = true;
  recvd_count_tof_cam_control++;
  resvdBytesFromTofCamControl = recvdMsg->data.size();

  std::cout << "\n\033[1;34mRECVD FROM TOPIC toTofCamTopic resvdBytesFromTofCamControl = \033[0m" 
      << resvdBytesFromTofCamControl << std::endl;
  std::cout << "recvd_count_tof_cam_control = " << recvd_count_tof_cam_control << std::endl;

  if (recvdMsg->data.size() == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
    for (int i = 0; i < recvdMsg->data.size(); i++){
      dataFromTofCamControl[i] = recvdMsg->data[i];
      printf("[%u]", dataFromTofCamControl[i]);
    }
    std::cout << std::endl;
  }
}

void Tof_cam::sendMsgToTofCamControl(){
  //отправка пакета в топик "fromTofCamTopic"
  std_msgs::ByteMultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = sizeof(dataToTofCamControl);
  msg.data.clear();

  send_count_tof_cam_control++;
  std::cout << "send_count_tof_cam_control = " << send_count_tof_cam_control << std::endl;
  std::cout << "\033[1;34mSEND T0 fromTofCamTopic: \033[0m";

  for (int i = 0; i < sizeof(dataToTofCamControl); i++) {
    printf("[%u]", dataToTofCamControl[i]);
    msg.data.push_back(dataToTofCamControl[i]);
  }
  std::cout << std::endl;
  
  toTofCamControlPub.publish(msg);
}

uint8_t Tof_cam::getCmdFromTofCamControl(){
  return dataFromTofCamControl[0];
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
  if(!depthFrame.empty()) cv::imshow("depthFrame", depthFrame);
}

void Tof_cam::getDepthFrame16C1(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame16C1;
  oni.getDepthFrame16C1(depthFrame16C1);
  if(!depthFrame16C1.empty()) cv::imshow("depthFrame16C1", depthFrame16C1);
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

    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
    return;
  }
  dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::error);
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

    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
    return;
  }
  dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::error);
}

void Tof_cam::publishDepthFrame16C1(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame16C1;
  oni.getDepthFrame16C1(depthFrame16C1);

  if(!depthFrame16C1.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_msg.image = depthFrame16C1;
    pubDepth.publish(out_msg.toImageMsg());

    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
    return;
  }
  dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::error);
}

void Tof_cam::publishIrFrame(){
  if (!oni.m_irStreamIsValid()) return;
  cv::Mat irFrame;
  oni.getIrFrame(irFrame);

  if(!irFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = irFrame;
    pubIr.publish(out_msg.toImageMsg());

    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
    return;
  }
  dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::error);
}

void Tof_cam::shutdownTofCam(){
  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
  }
}

void Tof_cam::resetTofCam(){

  std::cout << "here1\n";

  if (oni.getIsReset()){
    std::cout << "here2\n";
    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
    return;
  }

  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
  }

  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      printf("Initializatuion failed");
    }
    printf("oni.init()\n");
    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
  }

  oni.setIsReset(true);
}

void Tof_cam::turnOnTofCam(){
  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      printf("Initializatuion failed");
    }
    printf("oni.init()\n");
    dataToTofCamControl[0] = dataFromTofCamControl[0];
    dataToTofCamControl[1] = static_cast<int>(TofCamErrorStatus::allOk);
  }
}