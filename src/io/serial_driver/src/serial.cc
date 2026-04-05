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
  // 检查端口是否打开
  if (!port_.is_open()) {
    reopen(serial_name_, baud_rate_, max_try_);
    std::cerr << "端口未打开，无法接收数据" << std::endl;
    return false;
  }

  auto buffer = std::make_shared<std::vector<uint8_t>>(sizeof(ReceiveData));
  auto completed = std::make_shared<bool>(false);
  auto header = std::make_shared<uint8_t>();
  auto crc_valid = std::make_shared<bool>(false);

  // 重置 io_service
  io_.reset();

  // 设置超时定时器
  timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
  timer_.async_wait([completed, this](const boost::system::error_code &ec) {
    if (!ec && !(*completed) && port_.is_open()) {
      boost::system::error_code cancel_ec;
      port_.cancel(cancel_ec);
      if (cancel_ec) {
        std::cerr << "取消操作失败: " << cancel_ec.message() << std::endl;
      }
    }
  });

  // 读取第一个字节
  boost::asio::async_read(
      port_, boost::asio::buffer(header.get(), 1),
      [this, header, buffer, completed, &data, timeout_ms,
       crc_valid](const boost::system::error_code &ec, size_t) {
        if (!ec && port_.is_open() && *header == 'M') {
          // 读取第二个字节
          boost::asio::async_read(
              port_, boost::asio::buffer(header.get(), 1),
              [this, header, buffer, completed, &data, timeout_ms,
               crc_valid](const boost::system::error_code &ec, size_t) {
                if (!ec && port_.is_open() && *header == 'A') {
                  // 设置帧头
                  (*buffer)[0] = 'M';
                  (*buffer)[1] = 'A';
                  // 读取剩余数据
                  boost::asio::async_read(
                      port_,
                      boost::asio::buffer(buffer->data() + 2,
                                          sizeof(ReceiveData) - 2),
                      [buffer, completed, &data,
                       crc_valid](const boost::system::error_code &ec, size_t) {
                        if (!ec) {
                          // 反序列化数据
                          data.deserialize(buffer->data(), buffer->size());
                          if (data.verify()) {
                            *crc_valid = true;
                            *completed = true;
                          } else {
                            // CRC校验失败
                            std::cerr << "CRC校验失败" << std::endl;
                            std::cerr << "期望CRC: " << data.crc16 << std::endl;
                            std::cerr << "计算CRC: " << data.calculateCRC()
                                      << std::endl;
                            *completed = false;
                          }
                        } else {
                          std::cerr << "读取剩余数据失败: " << ec.message()
                                    << std::endl;
                        }
                      });
                } else {
                  std::cerr << "第二个字节不是 'A'，收到: 0x" << std::hex
                            << (int)*header << std::dec << std::endl;
                }
              });
        } else {
          if (ec) {
            std::cerr << "读取第一个字节失败: " << ec.message() << std::endl;
          } else if (*header != 'M') {
            std::cerr << "帧头错误，期望 'M'，收到: 0x" << std::hex
                      << (int)*header << std::dec << std::endl;
          }
        }
      });

  // 运行 io_service
  io_.run();
  io_.reset();

  // 取消定时器
  boost::system::error_code timer_ec;
  timer_.cancel(timer_ec);

  return *completed && *crc_valid;
}

bool SerialDriver::reopen(std::string serial_name, int baud_rate, int max_try) {
  boost::system::error_code ec; // 在这里声明 ec

  if (port_.is_open()) {
    // 先取消所有异步操作
    port_.cancel(ec);
    if (ec) {
      std::cerr << "取消端口操作失败: " << ec.message() << std::endl;
    }

    // 关闭端口
    port_.close(ec);
    if (ec) {
      std::cerr << "关闭端口失败: " << ec.message() << std::endl;
      return false;
    }
  }

  bool is_open = false;
  std::cout << "尝试重新打开端口：" << ec.message() << std::endl;
  for (int i = 0; i < max_try; i++) {
    is_open = open(serial_name, baud_rate);
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