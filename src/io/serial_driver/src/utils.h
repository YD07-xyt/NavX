#pragma once
#include"ros2/serial_node.h"
#include "serial/serial.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>
#include <iomanip>
namespace tool {
void printBuffer(const std::vector<uint8_t>& buffer) {
    std::cout << "Buffer size: " << buffer.size() << " bytes" << std::endl;
    std::cout << "Data: ";
    for (size_t i = 0; i < buffer.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(buffer[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

}