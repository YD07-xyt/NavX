#pragma once
#include "tools/logger.hpp"
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/system/error_code.hpp>
#include <cstring>
#include "packet_typedef.h"
#include <boost/circular_buffer.hpp>
#include <string>
namespace io {

class SerialDriver {
public:
    SerialDriver():
        io_(),
        io_context(),
        serial_port_(io_),
        socket_port_(io_context),
        serial_rx_buffer_(32768),
        socket_rx_buffer_(32768) {
        logger_ = logger::create_colored_logger("serial");
    };
    ~SerialDriver() {
        socket_port_.close();
        serial_port_.close();
    }
    bool open_socket(std::string receive_name, std::string send_name);
    bool open_serial(std::string serial_name, int baud_rate);
    bool reopen_serial(std::string serial_name, int baud_rate, int max_try);
    bool reopen_socket(std::string receive_name, std::string send_name, int max_try);
    bool send_socket(const SendSocketData& send_data);
    bool send_serial(const SendSerialData& send_data);
    void init_serial(std::string serial_name, int baud_rate, int max_try);
    void init_socket(std::string receive_name, std::string send_name,int max_try);
    //TODO：处理crc
    bool receive_all_serial(std::vector<ReceiveSerialData>& data, int timeout_ms);
    bool receive_all_socket(std::vector<ReceiveSocketData>& data, int timeout_ms);

private:
    //TODO：模板统一
    bool find_packet_in_buffer(std::vector<ReceiveSerialData>& data);
    bool find_packet_in_buffer_socket(std::vector<ReceiveSocketData>& data);

private:
    int max_try_;
    boost::system::error_code ec;
    boost::circular_buffer<uint8_t> serial_rx_buffer_;
    boost::circular_buffer<uint8_t> socket_rx_buffer_;
    std::mutex serial_buffer_mutex_;
    std::mutex socket_buffer_mutex_;

private:
    std::string serial_name_;
    int baud_rate_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;


private:
    std::string socket_receive_name_;
    std::string socket_send_name_;
    boost::asio::io_context io_context;
    boost::asio::local::datagram_protocol::socket socket_port_;

private:
    std::shared_ptr<spdlog::logger> logger_;
};
}