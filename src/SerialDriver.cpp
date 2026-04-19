#include "SerialDriver.h"
#include "crc.hpp" 
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
    if (fd != -1) {
        // 传入 const 引用，不能直接修改原数据的 crc 字段，所以拷贝一份
        SendPacket tx_pkt = pkt;
        
        // 计算发送端的 CRC16 校验码
        // 参数1：结构体的首地址
        // 参数2：需要计算的长度（总长度减去结构体末尾 crc16 占用的 2 个字节）
        tx_pkt.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&tx_pkt), sizeof(SendPacket) - 2);
        
        write(fd, &tx_pkt, sizeof(SendPacket));
    }
}

bool SerialDriver::receive_packet(ReceivePacket& in_pkt) {
    uint8_t buffer[sizeof(ReceivePacket)];
    uint8_t byte;

    // 1. 循环读取，直到匹配帧头
    while (read(fd, &byte, 1) > 0) {
        if (byte == 0x5A) {
            buffer[0] = 0x5A;
            // 2. 必须一次性读完剩下的字节，否则视为无效帧
            int total_read = 1;
            while (total_read < sizeof(ReceivePacket)) {
                int n = read(fd, buffer + total_read, sizeof(ReceivePacket) - total_read);
                if (n <= 0) return false; 
                total_read += n;
            }
            

            //check_crc16 函数自动取 buffer 最后两个字节与前面的数据计算结果进行比对
            if (!tools::check_crc16(buffer, sizeof(ReceivePacket))) {
                // 如果验证失败，说明通信出现误码或断帧，直接 return false 丢弃该包
                return false; 
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