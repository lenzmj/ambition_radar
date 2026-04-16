#include "SerialDriver.h"
#include <iostream>


SerialDriver::SerialDriver(const char* port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "[SerialDriver] Failed to open port " << port
                  << ": " << strerror(errno) << std::endl;
        return;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "[SerialDriver] tcgetattr failed for " << port
                  << ": " << strerror(errno) << std::endl;
        close(fd);
        fd = -1;
        return;
    }

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cerr << "[SerialDriver] tcsetattr failed for " << port
                  << ": " << strerror(errno) << std::endl;
        close(fd);
        fd = -1;
        return;
    }

    std::cout << "[SerialDriver] Opened port " << port << " successfully (fd="
              << fd << ")" << std::endl;
}

SerialDriver::~SerialDriver() {
    if (fd != -1) {
        close(fd);
        std::cout << "[SerialDriver] Port closed (fd=" << fd << ")" << std::endl;
    }
}

bool SerialDriver::isOpen() const {
    return fd != -1;
}

void SerialDriver::send_packet(const SendPacket& pkt) {
    if (fd == -1) {
        std::cerr << "[SerialDriver] send_packet failed: port not open" << std::endl;
        return;
    }

    ssize_t bytes_written = write(fd, &pkt, sizeof(SendPacket));
    if (bytes_written < 0) {
        std::cerr << "[SerialDriver] send_packet write error: "
                  << strerror(errno) << std::endl;
    } else if (static_cast<size_t>(bytes_written) != sizeof(SendPacket)) {
        std::cerr << "[SerialDriver] send_packet partial write: "
                  << bytes_written << "/" << sizeof(SendPacket)
                  << " bytes" << std::endl;
    }
}

bool SerialDriver::receive_packet(ReceivePacket& in_pkt) {
    if (fd == -1) {
        std::cerr << "[SerialDriver] receive_packet failed: port not open" << std::endl;
        return false;
    }

    uint8_t buffer[sizeof(ReceivePacket)];
    uint8_t byte;

    // 循环读取，直到匹配帧头
    while (true) {
        ssize_t rc = read(fd, &byte, 1);
        if (rc < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return false;
            }
            std::cerr << "[SerialDriver] receive_packet read error: "
                      << strerror(errno) << std::endl;
            return false;
        }
        if (rc == 0) {
            return false;
        }

        if (byte == 0x5A) {
            buffer[0] = 0x5A;
            // 必须一次性读完剩下的字节，否则视为无效帧
            size_t total_read = 1;
            while (total_read < sizeof(ReceivePacket)) {
                ssize_t n = read(fd, buffer + total_read, sizeof(ReceivePacket) - total_read);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        std::cerr << "[SerialDriver] receive_packet incomplete frame: "
                                  << total_read << "/" << sizeof(ReceivePacket)
                                  << " bytes (would block)" << std::endl;
                    } else {
                        std::cerr << "[SerialDriver] receive_packet read error during frame: "
                                  << strerror(errno) << std::endl;
                    }
                    return false;
                }
                if (n == 0) {
                    std::cerr << "[SerialDriver] receive_packet unexpected EOF during frame: "
                              << total_read << "/" << sizeof(ReceivePacket)
                              << " bytes" << std::endl;
                    return false;
                }
                total_read += n;
            }

            memcpy(&in_pkt, buffer, sizeof(ReceivePacket));
            return true;
        }
    }
}

void SerialDriver::flush_input() {
    if (fd == -1) {
        std::cerr << "[SerialDriver] flush_input failed: port not open" << std::endl;
        return;
    }
    // TCIFLUSH 表示清空收到的但还没读的数据
    if (tcflush(fd, TCIFLUSH) != 0) {
        std::cerr << "[SerialDriver] flush_input tcflush error: "
                  << strerror(errno) << std::endl;
    }
}
