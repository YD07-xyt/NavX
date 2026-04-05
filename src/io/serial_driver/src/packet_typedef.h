#pragma once
#include <cstdio>
#include <cstring>
#include<vector>
#include"crc.hpp"
#include <cstdint>
constexpr uint16_t SOF_VALUE = {'MA'};
namespace io {
    struct SendData{
        uint16_t sof;
        float v_x;
        float v_y;
        float w_z;
        
        uint16_t crc16;
        
        uint16_t calculateCRC16() const {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
            size_t len = offsetof(SendData, crc16);  // 只计算到 w_z 为止
            return crc16::get_CRC16_check_sum(data, len,crc16);
        }

        void init() {
            sof =SOF_VALUE;
            v_x = 0.0f;
            v_y = 0.0f;
            w_z = 0.0f;
            crc16 = 0;
            crc16 = calculateCRC16();
        }

        // 验证数据包
        bool verify() const {
            // 验证帧头
            if (sof != SOF_VALUE) {
                return false;
            }
            // 验证 CRC
            return calculateCRC16() == crc16;
        }
        
        // 序列化
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> buffer(sizeof(SendData));
            memcpy(buffer.data(), this, sizeof(SendData));
            return buffer;
        }
        
        // 反序列化
        bool deserialize(const uint8_t* data, size_t size) {
            if (size < sizeof(SendData)) return false;
            
            memcpy(this, data, sizeof(SendData));
            return verify();
        }
        
        // 设置速度
        void setVelocity(float vx, float vy, float wz) {
            v_x = vx;
            v_y = vy;
            w_z = wz;
            crc16 = calculateCRC16();  // 更新 CRC
        }
    };
    struct ReceiveData{
        uint16_t sof;
        uint8_t game_progress;
        uint16_t current_hp;
        uint16_t projectile_allowance;
        float vx;
        float vy;
        float wz;
        uint16_t crc16;
        
        void init() {
            sof = SOF_VALUE;
            game_progress = 0;
            current_hp = 0;
            projectile_allowance = 0;
            crc16 = 0;
            crc16 = calculateCRC();
        }
        
        uint16_t calculateCRC() const {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
            size_t len = offsetof(ReceiveData, crc16);
            return crc16::get_CRC16_check_sum(data, len,crc16);
        }
        
        bool verify() const {
            if (sof != SOF_VALUE){
                return false;
            }
            return calculateCRC() == crc16;
        }
        
        bool deserialize(const uint8_t* data, size_t size) {
            if (size < sizeof(ReceiveData)) return false;
            memcpy(this, data, sizeof(ReceiveData));
            return verify();
        }
        
        void print() const {
            printf("=== Robot Status ===\n");
            printf("SOF: %c%c\n", (sof >> 8) & 0xFF, sof & 0xFF);
            printf("Game: %s\n", game_progress ? "Running" : "Stopped");
            printf("HP: %u\n", current_hp);
            printf("Ammo: %u\n", projectile_allowance);
            printf("CRC: 0x%04X\n", crc16);
        }
    };
}