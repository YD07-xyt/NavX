#pragma once
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <vector>
#include <iomanip>
namespace tool {
void print_buffer(const std::vector<uint8_t>& buffer) {
    std::cout << "Buffer size: " << buffer.size() << " bytes" << std::endl;
    std::cout << "Data: ";
    for (unsigned char i : buffer) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(i) << " ";
    }
    std::cout << std::dec << std::endl;
}

}