#pragma once
#include "tof_cam.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Tof_cam::Tof_cam(){
  if (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed");
  }
  printf("oni.init()\n");
  
  pubColor  = nh.advertise<sensor_msgs::Image>(TO_COLOR_TOPIC, 1);
  pubDepth  = nh.advertise<sensor_msgs::Image>(TO_DEPTH_TOPIC, 1);
  pubIr     = nh.advertise<sensor_msgs::Image>(TO_IR_TOPIC, 1);
  commands_sub = nh.subscribe(FROM_COMMAND_TOPIC, 1, &Tof_cam::fromCommandTopicCallback, this);
  
  memset(commandFromTopic, 0, sizeof(commandFromTopic));
  commandFromTopic[3] = 3;
  countBmp            = 0;
  eth_recvd_count     = 0;
}

Tof_cam::~Tof_cam(){}

Tof_cam::Tof_cam(int a){}

void Tof_cam::fromCommandTopicCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) {
  eth_recvd_count++;
  std::cout << "\033[1;34m fromCommandTopicCallback \033[0m\n";
  std::cout << "eth_recvd_count         = " << eth_recvd_count << std::endl;
  std::cout << "recvd_msg->data.size()  = " << msg->data.size() << std::endl;
  for (int i = 0; i < msg->data.size(); i++){
    commandFromTopic[i] = (uint8_t)msg->data[i];
    printf("[%u]", commandFromTopic[i]);
  }
  std::cout << "\n";
}

uint8_t Tof_cam::getCommandFromTopic(){
  return commandFromTopic[3];
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

void Tof_cam::saveColorFrameMaxQuality(){
  countBmp++;
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrameMaxQuality;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1920_1080_RGB888_15FPS);
  std::cout << "publishColorFrameMaxQuality: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);
  cv::imwrite("colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp", colorFrameMaxQuality);
  commandFromTopic[3] = 4;
}

void Tof_cam::sendToTCPColorFrameMaxQuality(){
  boost::system::error_code ec;
  boost::asio::io_context context;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address("127.0.0.1"), 1234);    
  boost::asio::ip::tcp::socket socket(context);
  socket.connect(endpoint, ec);

  if (socket.is_open()) {
    std::string fileName = "colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp";
    socket.send(boost::asio::buffer(fileName.data(), fileName.size()));

    // Wait for response from server
    socket.wait(socket.wait_read);
    std::size_t bytes = socket.available();
    if (bytes > 0) {
      std::string response;
      response.resize(bytes);
      socket.read_some(boost::asio::buffer(response.data(), bytes), ec);
      if (response != "OK")
      {
        std::cerr << "Unexpected server Error!\n";
      }
    }

    std::ifstream input(fileName.data(), std::ios::binary);
    std::string buffer(std::istreambuf_iterator<char>(input), {});
    socket.send(boost::asio::buffer(buffer.data(), buffer.size()));
    input.close();
    std::cout << "File sent.\n";
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
  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
  }
}

void Tof_cam::resetTofCam(){

  if (oni.getIsReset()) return;

  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
  }

  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      printf("Initializatuion failed");
    }
    printf("oni.init()\n");
  }

  oni.setIsReset(true);
  commandFromTopic[3] = 3;
}

void Tof_cam::turnOnTofCam(){
  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      printf("Initializatuion failed");
    }
    printf("oni.init()\n");
  }
  commandFromTopic[3] = 3;
}