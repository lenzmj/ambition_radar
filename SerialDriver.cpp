#include "SerialDriver.h"
#include <iostream>


SerialDriver::SerialDriver(const char* port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tcsetattr(fd, TCSANOW, &options);
}

SerialDriver::~SerialDriver() { if (fd != -1) close(fd); }

void SerialDriver::send_packet(const SendPacket& pkt) {
    if (fd != -1) write(fd, &pkt, sizeof(SendPacket));
}

bool SerialDriver::receive_packet(ReceivePacket& in_pkt) {
    uint8_t buffer[sizeof(ReceivePacket)];
    uint8_t byte;

    // 1. 循环读取，直到匹配帧头
    while (read(fd, &byte, 1) > 0) {
        if (byte == 0x5A) {
            buffer[0] = 0x5A;
            // 2. 必须一次性读完剩下的 15 字节，否则视为无效帧
            int total_read = 1;
            while (total_read < sizeof(ReceivePacket)) {
                int n = read(fd, buffer + total_read, sizeof(ReceivePacket) - total_read);
                if (n <= 0) return false; 
                total_read += n;
            }
            

            memcpy(&in_pkt, buffer, sizeof(ReceivePacket));
            return true;
        }
    }
    return false;
}
void SerialDriver::flush_input() {
    if (fd != -1) {
        // TCIFLUSH 表示清空收到的但还没读的数据
        tcflush(fd, TCIFLUSH); 
    }
}