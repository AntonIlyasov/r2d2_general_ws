#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <stdio.h>
#include <string.h>
#include "umba_crc_table.h"
using boost::asio::ip::udp;
using boost::asio::ip::address;

#define PORT 1234
#define IP "127.0.0.1"

bool array = false;
class UDPClient
{
public:
    UDPClient(boost::asio::io_context& io_context)
    : io_context_(io_context), socket_(io_context){
        std::cout << "UDP CLIENT IS RUNNING\n";
        sender_endpoint_ = udp::endpoint(address::from_string(IP), PORT);
        socket_.open(udp::v4());
    }

    void sendMsg(uint8_t num) {
        auto start = std::chrono::high_resolution_clock::now();

        uint8_t msg[] =       {num, num + 1, num + 2};

        std::cout << "\nInput... ";
        
        static uint32_t send_count = 0;
        boost::system::error_code err;
        
        auto sent = socket_.send_to(boost::asio::buffer(msg), sender_endpoint_, 0, err);
        if (!err && sent > 0){
            std::cout << "\nSEND TO UDP: ";
            for (int i = 0; i < sizeof(msg); i++){
                printf("[%u]", msg[i]);
            }
            std::cout << std::endl;
            send_count++;
            std::cout << "Sent Payload = " << sent << "\n";
            std::cout << "send_count = " << send_count << "\n";
            //printf("[crc8 = %u\n]", crc8);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end - start;
        float mls = duration.count() * 1000;
        std::cout << "mls: " << mls << std::endl;
    }

    ~UDPClient() {
        socket_.close();
    }

private:
    boost::asio::io_context& io_context_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
};

int main(int argc, char* argv[])
{
    // uint8_t msg1[] = {0, 5, 112, 0};
    // uint8_t crc8_ = umba_crc8_table(msg1, sizeof(msg1));
    // std::cout << "crc8_ = ";
    // printf("%u\n", crc8_);
    // while(1){;}

    try{
        boost::asio::io_context io_context;
        UDPClient udpClient(io_context);
        uint8_t cnt = 0;
        while(1){
            udpClient.sendMsg(cnt);
            cnt++;
        }
    } catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
    }
    return 0;
}