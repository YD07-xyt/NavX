#include "serial/serial.h"
namespace io {
void SerialDriver::init_socket(std::string receive_name, std::string send_name, int max_try) {
    // 初始化套接字
    this->socket_receive_name_ = receive_name;
    this->socket_send_name_ = send_name;
    this->max_try_ = max_try;
}
bool SerialDriver::open_socket(std::string receive_name, std::string send_name) {
    this->socket_receive_name_ = receive_name;
    this->socket_send_name_ = send_name;
    logger::info(logger_, "socket_send_name:{},socket_receive_name:{}", send_name, receive_name);
    // 清理可能存在的旧 socket 文件
    // 尝试删除旧文件，忽略“文件不存在”错误
    if (::unlink(receive_name.c_str()) != 0 && errno != ENOENT) {
        logger::warn(logger_, "unlink {} failed: {}", receive_name, strerror(errno));
    }
    // 确保套接字已关闭
    if (socket_port_.is_open()) {
        socket_port_.close(ec);
        if (ec) {
            logger::warn(logger_, "close existing socket failed: {}", ec.message());
        }
    }
    //打开套接字
    socket_port_.open(boost::asio::local::datagram_protocol(), ec);
    if (ec) {
        logger::error(logger_, "open socket failed: {}", ec.message());
        return false;
    }
    //绑定本地地址
    boost::asio::local::datagram_protocol::endpoint ep_receive(receive_name);
    socket_port_.bind(ep_receive, ec);
    if (ec) {
        logger::error(logger_, "bind receive endpoint {} failed: {}", receive_name, ec.message());
        socket_port_.close();
        return false;
    }
    logger::info(logger_, "bind success on {}", receive_name);

    //连接远程地址
    boost::asio::local::datagram_protocol::endpoint ep_send(send_name);
    socket_port_.connect(ep_send, ec);
    if (ec) {
        logger::error(logger_, "connect to send endpoint {} failed: {}", send_name, ec.message());
        socket_port_.close();
        // 清理已绑定的文件
        ::unlink(receive_name.c_str());
        return false;
    }
    logger::info(logger_, "connect to {} success", send_name);
    return true;
}

bool SerialDriver::send_socket(const SendSocketData& send_data) {
    std::vector<uint8_t> buffer = send_data.serialize();

    size_t sent = socket_port_.send(boost::asio::buffer(buffer), 0, ec);
    if (ec) {
        logger::error(logger_, "Send socket error: {}", ec.message());
        return false;
    }

    if (sent != buffer.size()) {
        logger::error(logger_, "Partial send: {} of {} bytes", sent, buffer.size());
        return false;
    }
    return true;
}
bool SerialDriver::reopen_socket(std::string receive_name, std::string send_name, int max_try) {
    if (socket_port_.is_open()) {
       socket_port_.close(ec);
    }
    bool is_open = false;
    logger::info(logger_, "尝试重新打开端口：{}", ec.message());
    for (int i = 0; i < max_try; i++) {
        is_open = open_socket(receive_name, send_name);
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
};

// 读取一次当前可用的所有数据，解析出完整数据包
// 返回 true 表示至少解析出一个包，data 中包含所有成功解析的包
// 参数 timeout_ms 暂未使用（可扩展）
bool SerialDriver::receive_all_socket(std::vector<ReceiveSocketData>& data, int timeout_ms = 0) {
    std::lock_guard<std::mutex> lock(socket_buffer_mutex_);

    data.clear();

    // ---------- 1. 非阻塞读取一个 UDP 数据报 ----------
    uint8_t temp_buffer[32768]; // 足够容纳最大 UDP 数据报
    boost::system::error_code ec;
    size_t bytes_read = socket_port_.receive(
        boost::asio::buffer(temp_buffer, sizeof(temp_buffer)),
        0, // flags = 0 表示阻塞直到有数据，但这里我们放在循环外部，由调用者决定
        ec
    );

    if (ec == boost::asio::error::would_block) {
        // 没有数据可读（如果 socket 被设为非阻塞，但这里我们没有设置）
        return false;
    }
    if (ec) {
        logger::error(logger_, "UDP 接收错误: {}", ec.message());
        return false;
    }
    if (bytes_read == 0) {
        // UDP 对端关闭不会发生，但保留
        return false;
    }

    // ---------- 2. 将数据压入环形缓冲区 ----------
    for (size_t i = 0; i < bytes_read; ++i) {
        socket_rx_buffer_.push_back(temp_buffer[i]);
    }

    // ---------- 3. 从缓冲区解析数据包 ----------
    return find_packet_in_buffer_socket(data);
}
// 从环形缓冲区中提取所有完整的包
bool SerialDriver::find_packet_in_buffer_socket(std::vector<ReceiveSocketData>& data) {
    const size_t data_len = sizeof(ReceiveSocketData);
    if (socket_rx_buffer_.size() < data_len) {
        return false; // 数据不足一个包
    }

    size_t i = 0;
    size_t last_packet_end = 0; // 记录最后一个完整包的结束位置
    bool found_packet = false;
    std::vector<uint8_t> packet(data_len); // 临时存储

    // 遍历缓冲区，查找帧头 (SOF0, SOF1)
    while (i + data_len <= socket_rx_buffer_.size()) {
        if (socket_rx_buffer_[i] == SOF0 && socket_rx_buffer_[i + 1] == SOF1) {
            // 复制一个完整包
            for (size_t j = 0; j < data_len; ++j) {
                packet[j] = socket_rx_buffer_[i + j];
            }
            ReceiveSocketData new_data;
            if (new_data.deserialize(packet.data(), data_len)) {
                data.push_back(new_data);
                last_packet_end = i + data_len;
                found_packet = true;
            }
            i += data_len; // 跳过当前包
        } else {
            ++i; // 逐字节向前扫描
        }
    }

    // 如果找到了至少一个包，则移除已处理的数据（包括最后一个包）
    if (found_packet) {
        // 删除从开始到最后一个包结尾的所有数据
        socket_rx_buffer_.erase(socket_rx_buffer_.begin(), socket_rx_buffer_.begin() + last_packet_end);
        return true;
    }

    // 未找到任何包：为了不丢失可能跨边界的包头，只保留最后 (data_len-1) 个字节
    if (socket_rx_buffer_.size() > data_len) {
        size_t keep = data_len - 1;
        size_t remove = socket_rx_buffer_.size() - keep;
        socket_rx_buffer_.erase(socket_rx_buffer_.begin(), socket_rx_buffer_.begin() + remove);
    }
    return false;
}
}