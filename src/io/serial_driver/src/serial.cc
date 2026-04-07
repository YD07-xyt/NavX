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
    if (!reopen(serial_name_, baud_rate_, max_try_)) {
      std::cerr << "端口未打开，无法接收数据" << std::endl;
      return false;
    }
  }
  
  // 设置总超时
  auto start_time = std::chrono::steady_clock::now();
  boost::system::error_code ec;
  
  // 帧同步：最多尝试100次
  const int max_sync_attempts = 100;
  for (int attempt = 0; attempt < max_sync_attempts; attempt++) {
    // 检查是否超时
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
    if (elapsed >= timeout_ms) {
      std::cerr << "接收数据总超时" << std::endl;
      return false;
    }
    
    uint8_t byte;
    size_t bytes_read = boost::asio::read(port_, boost::asio::buffer(&byte, 1), ec);
    
    if (ec) {
      if (ec == boost::asio::error::eof) {
        std::cerr << "串口连接断开" << std::endl;
      } else if (ec == boost::asio::error::operation_aborted) {
        std::cerr << "读取操作被取消" << std::endl;
      } else {
        std::cerr << "读取失败: " << ec.message() << std::endl;
      }
      return false;
    }
    
    // 找到 'M'
    if (byte == 'M') {
      // 尝试读取下一个字节
      bytes_read = boost::asio::read(port_, boost::asio::buffer(&byte, 1), ec);
      
      if (ec) {
        std::cerr << "读取第二个字节失败: " << ec.message() << std::endl;
        return false;
      }
      
      // 检查是否为 'A'
      if (byte == 'A') {
        // 找到完整帧头，开始读取数据
        std::vector<uint8_t> buffer(sizeof(ReceiveData));
        buffer[0] = 'M';
        buffer[1] = 'A';
        
        // 读取剩余数据，剩余超时时间
        int remaining_timeout = timeout_ms - elapsed;
        if (remaining_timeout < 0) {
          return false;
        }
        
        // 设置剩余数据的读取超时
        boost::asio::deadline_timer data_timer(io_);
        data_timer.expires_from_now(boost::posix_time::milliseconds(remaining_timeout));
        bool data_timeout = false;
        data_timer.async_wait([&](const boost::system::error_code &ec) {
          if (!ec) data_timeout = true;
        });
        
        // 读取剩余数据
        bytes_read = boost::asio::read(port_, 
                                        boost::asio::buffer(buffer.data() + 2, 
                                                            sizeof(ReceiveData) - 2), 
                                        ec);
        data_timer.cancel();
        
        if (data_timeout) {
          std::cerr << "读取数据超时" << std::endl;
          continue;  // 重新同步
        }
        
        if (ec) {
          std::cerr << "读取数据失败: " << ec.message() << std::endl;
          continue;  // 重新同步
        }
        
        // 反序列化和校验
        data.deserialize(buffer.data(), buffer.size());
        if (data.verify()) {
          std::cout << "数据接收成功，CRC校验通过" << std::endl;
          return true;
        } else {
          std::cerr << "CRC校验失败，重新同步" << std::endl;
          continue;  // CRC失败，重新同步
        }
      } else {
        // 第二个字节不是'A'，继续查找
        std::cerr << "期望 'A'，收到: 0x" << std::hex << (int)byte 
                  << std::dec << "，重新同步" << std::endl;
        // 注意：当前字节可能是新的'M'，所以不退回到查找状态
        if (byte == 'M') {
          // 回退一个字节，让下一次循环处理
          continue;
        }
      }
    }
  }
  
  std::cerr << "帧同步失败，已达到最大尝试次数" << std::endl;
  return false;
}

bool SerialDriver::receive1(ReceiveData &data, int timeout_ms) {
  // 检查端口是否打开
  if (!port_.is_open()) {
    if (!reopen(serial_name_, baud_rate_, max_try_)) {
      std::cerr << "端口未打开，无法接收数据" << std::endl;
      return false;
    }
  }
  
  std::vector<uint8_t> buffer(sizeof(ReceiveData));
  
  // 设置超时（可选，取决于串口驱动）
  // 注意：boost::asio::read 本身没有超时参数，需要配合 deadline_timer
  uint8_t byte;
  // 一次性读取完整数据包
  size_t bytes_read = boost::asio::read(port_, boost::asio::buffer(buffer), ec);
 
  if (ec) {
    std::cerr << "读取失败: " << ec.message() << std::endl;
    return false;
  }
  
  if (bytes_read != sizeof(ReceiveData)) {
    std::cerr << "读取字节数不足，期望" << sizeof(ReceiveData) 
              << "字节，实际" << bytes_read << std::endl;
    return false;
  }
  if(buffer[0]=='M'&&buffer[1]=='A'){ 
      // 反序列化和校验
      data.deserialize(buffer.data(), buffer.size());
      
      if (!data.verify()) {
        std::cerr << "crc16数据校验失败" << std::endl;
        return false;
      }
  }
  return true;
}

bool SerialDriver::reopen(std::string serial_name, int baud_rate, int max_try) {

  if (port_.is_open()) {
    // 先取消所有异步操作
    //port_.cancel(ec);
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