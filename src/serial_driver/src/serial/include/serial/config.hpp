#pragma once

#include <string>
namespace io {
enum SendingMethod {
    SERIAL,
    SOCKET,
};
struct SerialConfig {
    std::string serial_name;
    int baud_rate;
    int max_try;
    std::string socket_send_name;
    std::string socket_receive_name;
    SendingMethod sending_method;
};
}