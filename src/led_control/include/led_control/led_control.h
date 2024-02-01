#pragma once
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

class Led_control{
public:
  // constructors & destructors
  Led_control();
  ~Led_control();
  
private:
  ros::NodeHandle _node;
  ros::Subscriber led_eth_receiver_sub;
  void led_eth_receiverCallback(const std_msgs::ByteMultiArray::ConstPtr& msg);
};