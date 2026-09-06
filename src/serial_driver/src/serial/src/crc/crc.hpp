#pragma once
#ifndef CRC_HPP
#define CRC_HPP
#include <cstdint>
#include <vector>
//#include"../../../tool/rm_log/include/rm_log.hpp"
namespace io {

namespace crc8 {
extern uint8_t get_CRC8_check_sum(uint8_t *pchMessage, unsigned int dwLength,
                                  uint8_t ucCRC8);

extern bool verify_CRC8_check_sum(uint8_t *pchMessage, unsigned int dwLength);

extern void append_CRC8_check_sum(uint8_t *pchMessage, unsigned int dwLength);
} // namespace crc8

namespace crc16 {
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength,
                                    uint16_t wCRC);

extern uint16_t get_CRC16_check_sum(const uint8_t *pchMessage, uint32_t dwLength,
                                    uint16_t wCRC);

extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

extern void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

// 对vector重载

extern bool verify_CRC16_check_sum(std::vector<uint8_t> &pchMessage);
} // namespace crc16

} // namespace io
#endif