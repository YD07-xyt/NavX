#pragma once

#include <cstring>
#include <vector>

#include <cstdint>
constexpr uint16_t SOF_VALUE = (('M' << 8) | 'B');
constexpr uint8_t SOF0 = 'M';
constexpr uint8_t SOF1 = 'B';
namespace io {
struct __attribute__((packed)) SendDataBase {
    uint8_t sof_0;
    uint8_t sof_1;
    uint8_t game_progress; // 比赛是否开始
    uint16_t current_hp; // 哨兵当前血量
    uint16_t projectile_allowance; // 哨兵可发弹量
    uint8_t is_enemy_outpost_destroyed; // 对方前哨站是否被摧毁
    uint16_t game_time; // 比赛时间（秒）
    float vx;
    float vy;
    float wz;
};

struct __attribute__((packed)) SendSerialData: public SendDataBase {
    uint16_t crc16;
    uint16_t calculate_crc16() const;
    // 验证数据包
    bool verify() const;

    // 序列化
    std::vector<uint8_t> serialize() const;

    // 反序列化
    bool deserialize(const uint8_t* data, size_t size);
};
struct __attribute__((packed)) SendSocketData: public SendDataBase {
    // 序列化
    std::vector<uint8_t> serialize() const;
};

struct __attribute__((packed)) ReceiveDataBase {
    uint8_t sof_0;
    uint8_t sof_1;
    uint8_t game_progress; // 比赛是否开始
    uint16_t current_hp; // 哨兵当前血量
    uint16_t projectile_allowance; // 哨兵可发弹量
    uint8_t is_enemy_outpost_destroyed; // 对方前哨站是否被摧毁
    uint16_t game_time; // 比赛时间（秒）
    float vx;
    float vy;
    float wz;
};

struct __attribute__((packed)) ReceiveSocketData: public ReceiveDataBase {
    bool deserialize(const uint8_t* data, size_t size);
};
struct __attribute__((packed)) ReceiveData: public ReceiveDataBase {
    uint16_t crc16; // 校验字段

    uint16_t calculate_crc() const;

    bool verify() const;

    bool deserialize(const uint8_t* data, size_t size);
};
}