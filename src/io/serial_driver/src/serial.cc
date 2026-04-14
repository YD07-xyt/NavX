#include "serial.h"
#include <boost/asio/local/datagram_protocol.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>

namespace io {
bool SerialDriver::open_socket(std::string serial_name) {

  boost::asio::local::datagram_protocol::endpoint ep(serial_name);
  port_.connect(ep,ec);
  //port_.bind(ep,ec);
  if (ec) {
    std::cerr << "打开端口失败: " << ec.message() << std::endl;
    return false;
  }
  std::cout << "打开端口success: " << ec.message() << std::endl;
  return true;
}

bool SerialDriver::open_serial(std::string serial_name, int baud_rate) {

  serial_port_.open(serial_name, ec);

  if (ec) {
    std::cerr << "打开端口失败: " << ec.message() << std::endl;
    return false;
  }
  ////波特率
  serial_port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
  //字符大小
  serial_port_.set_option(boost::asio::serial_port::character_size(8));
  //停止位，
  serial_port_.set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  serial_port_.set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  //流量控制，
  serial_port_.set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));

  return true;
}

bool SerialDriver::send(const SendData &send_data) {
  std::vector<uint8_t> buffer = send_data.serialize();

  //size_t sent = boost::asio::write(port_, boost::asio::buffer(buffer), ec);
  size_t sent= port_.send(boost::asio::buffer(buffer), 0,ec);
  if (ec) {
    std::cerr << "Send error: " << ec.message() << std::endl;
    return false;
  }
  return sent == buffer.size();
}
void printBuffer(const std::vector<uint8_t>& buffer) {
    std::cout << "Buffer size: " << buffer.size() << " bytes" << std::endl;
    std::cout << "Data: ";
    for (size_t i = 0; i < buffer.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(buffer[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

bool SerialDriver::send_serial(const SendData &send_data) {
  std::vector<uint8_t> buffer = send_data.serialize();

  size_t sent = boost::asio::write(serial_port_, boost::asio::buffer(buffer), ec);
  printBuffer(buffer);
  if (ec) {
    std::cerr << "Send error: " << ec.message() << std::endl;
    return false;
  }
  return sent == buffer.size();
}
bool SerialDriver::receive_all_serial(std::vector<ReceiveData> &data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (!serial_port_.is_open()) {
        if (!reopen(serial_name_, baud_rate_, max_try_)) {
            std::cerr << "端口未打开" << std::endl;
            return false;
        }
    }
    
    data.clear();  
    
    // 读取可用数据
    uint8_t temp_buffer[1024];
    size_t bytes_read = serial_port_.read_some(
        boost::asio::buffer(temp_buffer, sizeof(temp_buffer)), ec);
    
    if (ec && ec != boost::asio::error::would_block) {
        std::cerr << "读取失败: " << ec.message() << std::endl;
        return false;
    }
    
    // 将新数据推入环形缓冲区
    for (size_t i = 0; i < bytes_read; i++) {
        rx_buffer_.push_back(temp_buffer[i]);
    }
    
    // 从缓冲区解析数据包
    return find_packet_in_buffer(data);
}

bool SerialDriver::receive_all(std::vector<ReceiveSocketData> &data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // if (!port_.is_open()) {
    //     std::cerr << "端口未打开" << std::endl;
    //     return false;
    // }
    
    data.clear();  
    
    // 读取可用数据
    uint8_t temp_buffer[1024];
    size_t bytes_read = port_.receive(
        boost::asio::buffer(temp_buffer, sizeof(temp_buffer)),0, ec);
    
    if (ec && ec != boost::asio::error::would_block) {
        std::cerr << "读取失败: " << ec.message() << std::endl;
        return false;
    }
    
    // 将新数据推入环形缓冲区
    for (size_t i = 0; i < bytes_read; i++) {
        rx_buffer_.push_back(temp_buffer[i]);
    }
    
    // 从缓冲区解析数据包
    return find_packet_in_buffer_socket(data);
}

bool SerialDriver::find_packet_in_buffer_socket(std::vector<ReceiveSocketData> &data) {
  if (rx_buffer_.size() < sizeof(ReceiveSocketData)) {
    return false; // 数据不足
  }

  bool found_packet = false;
  size_t i = 0;

  // 遍历环形缓冲区查找包头
  while (i <= rx_buffer_.size() - sizeof(ReceiveSocketData)) {
    if (rx_buffer_[i] == SOF0 && rx_buffer_[i + 1] == SOF1) {
      // 找到包头，提取数据
      uint8_t packet[sizeof(ReceiveSocketData)];
      for (size_t j = 0; j < sizeof(ReceiveSocketData); j++) {
        packet[j] = rx_buffer_[(i + j) % rx_buffer_.capacity()];
      }

      ReceiveSocketData new_data;
      new_data.deserialize(packet, sizeof(ReceiveSocketData));

      data.push_back(new_data); 

      i += sizeof(ReceiveSocketData);
      found_packet = true;
    } else {
      i++;
    }
  }


  if (found_packet) {

    size_t last_packet_end = 0;
    i = 0;
    while (i <= rx_buffer_.size() - sizeof(ReceiveSocketData)) {
      if (rx_buffer_[i] == 'M' && rx_buffer_[i + 1] == 'A') {
        last_packet_end = i + sizeof(ReceiveSocketData);
        i += sizeof(ReceiveSocketData);
      } else {
        i++;
      }
    }


    if (last_packet_end > 0) {
      rx_buffer_.erase_begin(last_packet_end);
    }

    return true;
  }


  if (rx_buffer_.size() > sizeof(ReceiveSocketData)) {
    size_t keep = sizeof(ReceiveSocketData) - 1;
    size_t remove = rx_buffer_.size() - keep;
    rx_buffer_.erase_begin(remove);
  }
  std::cerr << "读取good " << std::endl;
  return false;
}

bool SerialDriver::find_packet_in_buffer(std::vector<ReceiveData> &data) {
  if (rx_buffer_.size() < sizeof(ReceiveData)) {
    return false; // 数据不足
  }

  bool found_packet = false;
  size_t i = 0;

  // 遍历环形缓冲区查找包头
  while (i <= rx_buffer_.size() - sizeof(ReceiveData)) {
    if (rx_buffer_[i] == SOF0 && rx_buffer_[i + 1] == SOF1) {
      // 找到包头，提取数据
      uint8_t packet[sizeof(ReceiveData)];
      for (size_t j = 0; j < sizeof(ReceiveData); j++) {
        packet[j] = rx_buffer_[(i + j) % rx_buffer_.capacity()];
      }

      ReceiveData new_data;
      new_data.deserialize(packet, sizeof(ReceiveData));
      if(new_data.crc16!= new_data.calculateCRC()){
        std::cerr<<"crc16 error"<<std::endl;
      }
      data.push_back(new_data); 

      i += sizeof(ReceiveData);
      found_packet = true;
    } else {
      i++;
    }
  }


  if (found_packet) {

    size_t last_packet_end = 0;
    i = 0;
    while (i <= rx_buffer_.size() - sizeof(ReceiveData)) {
      if (rx_buffer_[i] == 'M' && rx_buffer_[i + 1] == 'A') {
        last_packet_end = i + sizeof(ReceiveData);
        i += sizeof(ReceiveData);
      } else {
        i++;
      }
    }


    if (last_packet_end > 0) {
      rx_buffer_.erase_begin(last_packet_end);
    }

    return true;
  }


  if (rx_buffer_.size() > sizeof(ReceiveData)) {
    size_t keep = sizeof(ReceiveData) - 1;
    size_t remove = rx_buffer_.size() - keep;
    rx_buffer_.erase_begin(remove);
  }

  return false;
}

bool SerialDriver::reopen(std::string serial_name, int baud_rate, int max_try) {

  if (serial_port_.is_open()) {
    // if (ec) {
    //   std::cerr << "取消端口操作失败: " << ec.message() << std::endl;
    // }
    // // 关闭端口
    // port_.close(ec);
    // if (ec) {
    //   std::cerr << "关闭端口失败: " << ec.message() << std::endl;
    //   return false;
    // }
    return true;
  }

  bool is_open = false;
  std::cout << "尝试重新打开端口：" << ec.message() << std::endl;
  for (int i = 0; i < max_try; i++) {
    is_open = open_serial(serial_name, baud_rate);
    if (is_open) {
      std::cout << "重新打开端口成功：" << ec.message() << std::endl;
      break;
    }
    if (i == (max_try - 1)) {
      std::cout << "尝试重新打开端口失败" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return is_open;
}
} // namespace io