#include"serial.h"

//TODO:
io::SendData get_send_data() {
    io::SendData data;
    data.init();
    data.sof = 0xAA;
    data.v_x = 1.0f;
    data.v_y = 0.5f;
    data.w_z = 0.2f;
    data.crc16 = data.calculateCRC16();
    return data;
}

// 实现 send_status()
bool send_status() {
    std::cout << "Status sent" << std::endl;
    return true;
}
//TODO: 串口重连
int main(){
    std::string serial_name="/dev/ttyUSB0";
    int baud_rate =115200;
    int max_try=1000000;

    io::SendData send_data=get_send_data();
    io::ReceiveData status;
    io::SerialDriver serial_driver;
    bool is_open;
    while(1){
        is_open=serial_driver.open(serial_name, baud_rate);
        if(is_open){
            serial_driver.reopen(serial_name,baud_rate,max_try);
        }

        serial_driver.receive(status,2000);
        
        send_status();

        serial_driver.send(send_data);
    }
    return 0;
}