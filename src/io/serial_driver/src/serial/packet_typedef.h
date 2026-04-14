#pragma once
#include <cstdio>
#include <cstring>
#include<vector>
#include"crc.hpp"
#include <cstdint>
constexpr uint16_t SOF_VALUE = (('M' << 8) | 'B');
constexpr uint8_t SOF0 = 'M';
constexpr uint8_t SOF1 = 'B';
namespace io {
   
    struct  __attribute__((packed))  SendData{
        //uint16_t sof;
        uint8_t sof_0;
        uint8_t sof_1;
        //uint8_t sentry_pose;
        float v_x;
        float v_y;
        float w_z;
        
        uint16_t crc16;
        
        uint16_t calculateCRC16() const {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
            size_t len = offsetof(SendData, crc16);  // 只计算到 w_z 为止
            return crc16::get_CRC16_check_sum(data, len,0xffff);
        }
        // 验证数据包
        bool verify() const {
            // 验证帧头
            if (sof_0 != SOF0||sof_1!=SOF1) {
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
    struct  __attribute__((packed))  ReceiveSocketData{
         //uint16_t sof;
        uint8_t sof_0;
        uint8_t sof_1;

        uint8_t game_progress; //比赛是否开始 开始：1 / 未开始：0
        uint16_t current_hp; //哨兵当前血量
        uint16_t projectile_allowance; //哨兵可发弹量
        float vx; // 当前速度
        float vy; 
        float wz;
    
     
        bool deserialize(const uint8_t* data, size_t size) {
            if (size < sizeof(ReceiveSocketData)) return false;
            memcpy(this, data, sizeof(ReceiveSocketData));
            return true;
        }
    };

    struct  __attribute__((packed))  ReceiveData{
         //uint16_t sof;
        uint8_t sof_0;
        uint8_t sof_1;

        uint8_t game_progress; //比赛是否开始 开始：1 / 未开始：0
        uint16_t current_hp; //哨兵当前血量
        uint16_t projectile_allowance; //哨兵可发弹量
        float vx; // 当前速度
        float vy; 
        float wz;
        uint16_t crc16;
            
        uint16_t calculateCRC() const {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
            size_t len = offsetof(ReceiveData, crc16);
            return crc16::get_CRC16_check_sum(data, len, 0xffff);
        }
        
        bool verify() const {
            if (sof_0 != SOF0||sof_1!=SOF1){
                return false;
            }
            return calculateCRC() == crc16;
        }
        
        bool deserialize(const uint8_t* data, size_t size) {
            if (size < sizeof(ReceiveData)) return false;
            memcpy(this, data, sizeof(ReceiveData));
            return verify();
        }
        
        // void print() const {
        //     printf("=== Robot Status ===\n");
        //     printf("SOF: %c%c\n", (sof >> 8) & 0xFF, sof & 0xFF);
        //     printf("Game: %s\n", game_progress ? "Running" : "Stopped");
        //     printf("HP: %u\n", current_hp);
        //     printf("Ammo: %u\n", projectile_allowance);
        //     printf("CRC: 0x%04X\n", crc16);
        // }
    };
}