#include "serial/serial.h"
#include <string>

namespace io {
void SerialDriver::init_serial(std::string serial_name, int baud_rate, int max_try) {
    // 初始化串口
    this->serial_name_ = serial_name;
    this->baud_rate_ = baud_rate;
    this->max_try_ = max_try;
}

bool SerialDriver::open_serial(std::string serial_name, int baud_rate) {
    serial_port_.open(serial_name, ec);

    if (ec) {
        logger::error(logger_, "打开端口失败: {}", ec.message());
        return false;
    }
    ////波特率
    serial_port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
    //字符大小
    serial_port_.set_option(boost::asio::serial_port::character_size(8));
    //停止位，
    serial_port_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    //流量控制，
    serial_port_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

    return true;
}

bool SerialDriver::send_serial(const SendSerialData& send_data) {
    std::vector<uint8_t> buffer = send_data.serialize();

    size_t sent = boost::asio::write(serial_port_, boost::asio::buffer(buffer), ec);
    //`tool::print_buffer(buffer);
    if (ec) {
        logger::error(logger_, "Send error: {}", ec.message());
        return false;
    }
    return sent == buffer.size();
}

bool SerialDriver::receive_all_serial(std::vector<ReceiveSerialData>& data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(serial_buffer_mutex_);

    if (!serial_port_.is_open()) {
        if (!reopen_serial(serial_name_, baud_rate_, max_try_)) {
            logger::error(logger_, "端口未打开");
            return false;
        }
    }

    data.clear();

    uint8_t temp_buffer[1024];
    size_t bytes_read = serial_port_.read_some(boost::asio::buffer(temp_buffer, sizeof(temp_buffer)), ec);

    if (ec && ec != boost::asio::error::would_block) {
        logger::error(logger_, "读取失败: {}", ec.message());
        return false;
    }

    // 即使 bytes_read == 0 也可能是因为超时或无数据，此时缓冲区未变，继续解析
    for (size_t i = 0; i < bytes_read; ++i) {
        serial_rx_buffer_.push_back(temp_buffer[i]);
    }

    // 无论是否读到新数据，都尝试从现有缓冲区解析数据包
    return find_packet_in_buffer(data);
}
bool SerialDriver::find_packet_in_buffer(std::vector<ReceiveSerialData>& data) {
    const size_t data_len = sizeof(ReceiveSerialData);
    if (serial_rx_buffer_.size() < data_len) {
        return false; // 缓冲区数据不足一个完整包
    }

    size_t i = 0;
    size_t last_packet_end = 0; // 记录最后一个成功解析的包的结束位置
    bool found_any = false;
    std::vector<uint8_t> packet(data_len); // 动态分配，替代 VLA

    // 单次遍历，边找包边记录最后包位置
    while (i + data_len <= serial_rx_buffer_.size()) {
        if (serial_rx_buffer_[i] == SOF0 && serial_rx_buffer_[i + 1] == SOF1) {
            // 提取完整包（直接拷贝连续内存）
            for (size_t j = 0; j < data_len; ++j) {
                packet[j] = serial_rx_buffer_[i + j]; // 无需取模，因为 i+j < size()
            }

            ReceiveSerialData new_data;
            if (new_data.deserialize(packet.data(), data_len)) {
                // CRC 校验
                if (new_data.crc16 != new_data.calculate_crc()) {
                    logger::error(logger_, "CRC16 错误，丢弃该包");
                } else {
                    data.push_back(new_data);
                    last_packet_end = i + data_len;
                    found_any = true;
                }
            }
            i += data_len; // 跳过当前包头部，继续往后找
        } else {
            ++i; // 逐字节滑动
        }
    }

    // 如果找到了至少一个有效包，则丢弃已处理的数据（从开头到最后一个包的结尾）
    if (found_any) {
        serial_rx_buffer_.erase_begin(last_packet_end);
        return true;
    }

    // 未找到任何包：为了防止跨边界的包头丢失，仅保留最后 (data_len - 1) 个字节
    if (serial_rx_buffer_.size() > data_len) {
        size_t keep = data_len - 1;
        size_t remove = serial_rx_buffer_.size() - keep;
        serial_rx_buffer_.erase_begin(remove);
    }
    return false;
}
bool SerialDriver::reopen_serial(std::string serial_name, int baud_rate, int max_try) {
    if (serial_port_.is_open()) {
        serial_port_.close(ec);
    }

    bool is_open = false;
    logger::info(logger_, "尝试重新打开端口：{}", ec.message());
    for (int i = 0; i < max_try; i++) {
        is_open = open_serial(serial_name, baud_rate);
        if (is_open) {
            logger::info(logger_, "重新打开端口成功：{}", ec.message());
            break;
        }
        if (i == (max_try - 1)) {
            logger::error(logger_, "尝试重新打开端口失败");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return is_open;
}
}