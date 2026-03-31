#include"serial.h"
#include <cstdint>
#include <vector>

namespace io {
    bool  SerialDriver::open(std::string serial_name,int baud_rate){
        boost::system::error_code ec;
        port_.open(serial_name, ec); 
        
        if(ec) {
            std::cerr << "打开端口失败: " << ec.message() << std::endl;
            return false;
        }
        ////波特率
        port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        //字符大小
        port_.set_option(boost::asio::serial_port::character_size(8));
        //停止位，
        port_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        //流量控制，
        port_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        return true;
    }

    bool SerialDriver::send(const SendData & send_data){
        std::vector<uint8_t> buffer = send_data.serialize();
        boost::system::error_code ec;
        
        size_t sent = boost::asio::write(port_, 
            boost::asio::buffer(buffer), ec);
        if (ec) {
            std::cerr << "Send error: " << ec.message() << std::endl;
            return false;
        }
        return sent == buffer.size();
    }
    bool SerialDriver::receive(ReceiveData& data, int timeout_ms){
        std::vector<uint8_t> buffer(sizeof(ReceiveData));
              // 等待帧头 'M'
        uint8_t byte;
        while (true) {
            boost::asio::read(port_, boost::asio::buffer(&byte, 1));
            if (byte == 'M') break;
        }
        
        // 读取第二个字节 'A'
        boost::asio::read(port_, boost::asio::buffer(&byte, 1));
        if (byte != 'A') {
            return false;
        }
        // 读取剩余数据
        buffer[0] = 'M';
        buffer[1] = 'A';
        boost::asio::read(port_, boost::asio::buffer(buffer.data() + 2, 
                         sizeof(ReceiveData) - 2));
        
        return data.deserialize(buffer.data(), buffer.size());
    }
}