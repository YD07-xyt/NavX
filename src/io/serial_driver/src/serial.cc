#include "serial.h"
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace io {
bool SerialDriver::open(std::string serial_name, int baud_rate) {
  boost::system::error_code ec;
  port_.open(serial_name, ec);

  if (ec) {
    std::cerr << "打开端口失败: " << ec.message() << std::endl;
    return false;
  }
  ////波特率
  port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
  //字符大小
  port_.set_option(boost::asio::serial_port::character_size(8));
  //停止位，
  port_.set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  port_.set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  //流量控制，
  port_.set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));

  return true;
}

bool SerialDriver::send(const SendData &send_data) {
  std::vector<uint8_t> buffer = send_data.serialize();

  size_t sent = boost::asio::write(port_, boost::asio::buffer(buffer), ec);
  if (ec) {
    std::cerr << "Send error: " << ec.message() << std::endl;
    return false;
  }
  return sent == buffer.size();
}


bool SerialDriver::receive(ReceiveData &data, int timeout_ms) {
  auto buffer = std::make_shared<std::vector<uint8_t>>(sizeof(ReceiveData));
  auto completed = std::make_shared<bool>(false);
  auto header = std::make_shared<uint8_t>();

  timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
  timer_.async_wait([completed, this](const boost::system::error_code &ec) {
    if (!ec && !(*completed)) {
      port_.cancel();
    }
  });

  // 读取第一个字节
  boost::asio::async_read(
      port_, boost::asio::buffer(header.get(), 1),
      [this, header, buffer, completed,
       &data](const boost::system::error_code &ec, size_t) {
        if (!ec && *header == 'M') {
          // 读取第二个字节
          boost::asio::async_read(
              port_, boost::asio::buffer(header.get(), 1),
              [this, header, buffer, completed,
               &data](const boost::system::error_code &ec, size_t) {
                if (!ec && *header == 'A') {
                  // 设置帧头
                  (*buffer)[0] = 'M';
                  (*buffer)[1] = 'A';
                  // 读取剩余数据
                  boost::asio::async_read(
                      port_,
                      boost::asio::buffer(buffer->data() + 2,
                                          sizeof(ReceiveData) - 2),
                      [buffer, completed,
                       &data](const boost::system::error_code &ec, size_t) {
                        if (!ec) {
                          *completed = true;
                          data.deserialize(buffer->data(), buffer->size());
                        }
                      });
                }
              });
        }
      });

  io_.run();
  io_.reset();
  timer_.cancel();
  return *completed;
}

bool SerialDriver::reopen(std::string serial_name, int baud_rate, int max_try) {
  if (port_.is_open()) {
    // 步骤2：关闭系统句柄
    port_.close(ec);
    if (ec) {
      std::cerr << "关闭端口失败: " << ec.message() << std::endl;
    }
  }
  bool is_open;
  for (int i = 0; i < max_try; i++) {
    is_open = open(serial_name, baud_rate);
    if (is_open) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return is_open;
}
} // namespace io