#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <cerrno>
#include "Protocol.h"

class SerialDriver {
public:
    SerialDriver(const char* port);
    ~SerialDriver();
    void send_packet(const SendPacket& pkt); 
    bool receive_packet(ReceivePacket& in_pkt);
    void flush_input();
    bool isOpen() const;

    // CRC8 校验 (多项式 0x07, 查表法)
    static uint8_t crc8(const uint8_t* data, size_t len);

private:
    int fd = -1;
    static const uint8_t crc8_table[256];
};

#endif // SERIAL_DRIVER_H
