#pragma  once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>
#include <cstring>
#include"packet_typedef.h"

namespace io {


    class SerialDriver{
        public:
            SerialDriver(std::string serial_name, int baud_rate, int max_try): 
                io_(), port_(io_),timer_(io_),
                serial_name_(serial_name),baud_rate_(baud_rate),max_try_(max_try){
            };
            ~SerialDriver(){
               port_.close(); 
            }
            bool open(std::string serial_name, int baud_rate);
            bool reopen(std::string serial_name,int baud_rate,int max_try);
            bool receive(ReceiveData& data, int timeout_ms = 1000) ;
            bool receive1(ReceiveData& data, int timeout_ms = 1000) ;
            bool send(const SendData & send_data);
            void init();
        private:
            std::string serial_name_;
            int baud_rate_;
            int max_try_;
            boost::system::error_code ec;
            boost::asio::io_service io_;
            boost::asio::serial_port port_;
            boost::asio::deadline_timer timer_; 
        };
}