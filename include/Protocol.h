#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <cstdint>

#pragma pack(push, 1)
struct SendPacket {
    uint8_t header = 0xA5; 
    uint8_t mode = 0;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float distance = 0.0f;
    uint8_t reserved = 0; // 补齐 16 字节
    uint8_t crc = 0;      // CRC8 校验
};

struct ReceivePacket {
    uint8_t header = 0x5A; // 接收帧头
    float current_yaw = 0.0f;
    float current_pitch = 0.0f;
    float current_roll = 0.0f; 
    uint8_t reserved[2] = {0, 0}; // 补齐 16 字节
    uint8_t crc = 0;              // CRC8 校验
};
#pragma pack(pop)
#endif
