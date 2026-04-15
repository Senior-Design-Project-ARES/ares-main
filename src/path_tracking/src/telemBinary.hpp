#pragma once

#include <cstdint>
#include <string>

namespace telem_binary
{

#pragma pack(push, 1)
struct TelemFrame
{
    uint16_t magic;
    uint16_t version;
    uint32_t seq;
    double t_s;
    float vx;
    float vy;
    float yaw_rate;
    uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(TelemFrame) == 30, "unexpected TelemFrame size");

uint16_t crc16_ccitt(const uint8_t *data, size_t len);
void encode_inplace(TelemFrame &frame);
bool decode_check(const TelemFrame &frame);
std::string frame_to_hex(const TelemFrame &frame);
TelemFrame encode_control_pair(double axial_vel, double turning_rate);

std::string encode_control_pair_to_hex(double axial_vel, double turning_rate);

}  // namespace telem_binary
