#include <iostream>
#include <ros/ros.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>

using boost::asio::ip::udp;
using boost::asio::ip::address;

#define PORT 1234
#define PACK_SIZE 4

ros::Publisher pub;

class UDPServer{
public:
  UDPServer(boost::asio::io_service& io_service)
  : socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    std::cout << "UDP SERVER IS RUNNING\n";
    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred);
    read_msg_udp();
  }

  void read_msg_udp(){
  socket_.async_receive_from(boost::asio::buffer(data_, sizeof(data_)), sender_endpoint_,
      boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
      boost::asio::placeholders::bytes_transferred));
  }

  void send_msg(size_t bytes_transferred){
    std_msgs::ByteMultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 1;
    msg.layout.dim[0].stride = bytes_transferred;
    msg.data.clear();

    for (int i = 0; i < bytes_transferred; i++) {
        msg.data.push_back(data_[i]);
    }
    
    std::cout << "\nSEND TO TOPIC ...\n";
    pub.publish(msg);
  }

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum {max_length = 60};
  uint8_t data_[max_length] = {0};
  uint32_t recvd_count = 0;

  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
      std::cout << "Receive failed: " << error.message() << "\n";
      return;
    } else {
      recvd_count++;
      std::cout << "recvd_count       = " << recvd_count << std::endl;
      std::cout << "bytes_transferred = " << bytes_transferred << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", data_[i]);
      }
      read_msg_udp();
      send_msg(bytes_transferred);
    }
  }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "led_eth_receiver_node");
    ros::NodeHandle node;
    pub = node.advertise<std_msgs::ByteMultiArray>("from_led_eth_receiver", 0);

    try{
      boost::asio::io_service io_service;
      UDPServer server(io_service);
      while(ros::ok()){
        io_service.poll_one();
        ros::spinOnce();
      }
    } catch (std::exception e){
		  std::cerr << "Exeption: " << e.what() << std::endl;
    }
    return 0;
}