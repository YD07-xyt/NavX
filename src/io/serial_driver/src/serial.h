#pragma  once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>
#include <cstring>
#include"packet_typedef.h"
#include <boost/circular_buffer.hpp>
namespace io {


    class SerialDriver{
        public:
            SerialDriver(std::string serial_name, int baud_rate, int max_try): 
                io_(),io_context(), port_(io_context),timer_(io_),
                serial_name_(serial_name),baud_rate_(baud_rate),max_try_(max_try){
            };
            ~SerialDriver(){
               port_.close(); 
            }
            bool open_socket(std::string serial_name, int baud_rate);
            bool open_serial(std::string serial_name, int baud_rate);
            bool reopen(std::string serial_name,int baud_rate,int max_try);
            //bool receive(ReceiveData& data, int timeout_ms = 1000) ;
            bool send(const SendData & send_data);
            void init();
            bool find_packet_in_buffer(std::vector<ReceiveData> & data);
            bool receive_all(std::vector<ReceiveData> &data, int timeout_ms);
        private:
            std::string serial_name_;
            int baud_rate_;
            int max_try_;
            boost::system::error_code ec;
            boost::asio::io_service io_;
            boost::asio::io_context io_context;
            //boost::asio::serial_port port_;
            boost::asio::local::stream_protocol::socket port_;
            boost::circular_buffer<uint8_t> rx_buffer_; 
            boost::asio::deadline_timer timer_; 
            std::mutex buffer_mutex_;  
        };
}