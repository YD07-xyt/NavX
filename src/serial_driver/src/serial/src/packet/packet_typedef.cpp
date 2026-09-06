#include "serial/packet_typedef.h"
#include "crc/crc.hpp"
#include <cstddef>
namespace io {
uint16_t SendSerialData::calculate_crc16() const {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
    size_t len = offsetof(SendSerialData, crc16); // 只计算到 w_z 为止
    return crc16::get_CRC16_check_sum(data, len, 0xffff);
}
bool SendSerialData::verify() const {
    // 验证帧头
    if (sof_0 != SOF0 || sof_1 != SOF1) {
        return false;
    }
    // 验证 CRC
    return calculate_crc16() == crc16;
}

// 序列化
std::vector<uint8_t> SendSerialData::serialize() const {
    std::vector<uint8_t> buffer(sizeof(SendSerialData));
    memcpy(buffer.data(), this, sizeof(SendSerialData));
    return buffer;
}

// 反序列化
bool SendSerialData::deserialize(const uint8_t* data, size_t size) {
    if (size < sizeof(SendSerialData)) return false;

    memcpy(this, data, sizeof(SendSerialData));
    return verify();
}

// 序列化
std::vector<uint8_t> SendSocketData::serialize() const {
    std::vector<uint8_t> buffer(sizeof(SendSerialData));
    memcpy(buffer.data(), this, sizeof(SendSerialData));
    return buffer;
}

bool ReceiveSocketData::deserialize(const uint8_t* data, size_t size) {
    if (size < sizeof(ReceiveSocketData)) return false;
    memcpy(this, data, sizeof(ReceiveSocketData));
    return true;
}
uint16_t ReceiveData::calculate_crc() const {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
    size_t len = offsetof(ReceiveData, crc16);
    return crc16::get_CRC16_check_sum(data, len, 0xffff);
}

bool ReceiveData::verify() const {
    if (sof_0 != SOF0 || sof_1 != SOF1) {
        return false;
    }
    return calculate_crc() == crc16;
}

bool ReceiveData::deserialize(const uint8_t* data, size_t size) {
    if (size < sizeof(ReceiveData)) return false;
    memcpy(this, data, sizeof(ReceiveData));
    return verify();
}
}