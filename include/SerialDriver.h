#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include "Protocol.h"

class SerialDriver {
public:
    SerialDriver(const char* port);
    ~SerialDriver();
    void send_packet(const SendPacket& pkt); 
    bool receive_packet(ReceivePacket& in_pkt);
    void flush_input();
    bool is_open() const { return fd != -1; }

private:
    int fd;
};

#endif // SERIAL_DRIVER_H
