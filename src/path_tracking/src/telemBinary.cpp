#include "telemBinary.hpp"

#include <iomanip>
#include <sstream>

namespace telem_binary
{

namespace
{
constexpr uint16_t kMagic = 0xA5A5U;
constexpr uint16_t kVersion = 1U;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void encode_inplace(TelemFrame &frame)
{
    frame.magic = kMagic;
    frame.version = kVersion;
    frame.crc = 0;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&frame);
    frame.crc = crc16_ccitt(bytes, sizeof(frame) - sizeof(frame.crc));
}

bool decode_check(const TelemFrame &frame)
{
    if (frame.magic != kMagic || frame.version != kVersion)
    {
        return false;
    }

    TelemFrame copy = frame;
    const uint16_t got = copy.crc;
    copy.crc = 0;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&copy);
    const uint16_t expect = crc16_ccitt(bytes, sizeof(copy) - sizeof(copy.crc));
    return got == expect;
}

std::string frame_to_hex(const TelemFrame &frame)
{
    const auto *encoded = reinterpret_cast<const uint8_t *>(&frame);
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < sizeof(frame); ++i)
    {
        if (i > 0)
        {
            oss << ' ';
        }
        oss << std::setw(2) << static_cast<unsigned int>(encoded[i]);
    }
    return oss.str();
}

TelemFrame encode_control_pair(double axial_vel, double turning_rate)
{
    TelemFrame frame{};
    frame.seq = 0;
    frame.t_s = 0.0;
    frame.vx = static_cast<float>(axial_vel);
    frame.vy = 0.0F;
    frame.yaw_rate = static_cast<float>(turning_rate);
    encode_inplace(frame);
    return frame;
}

std::string encode_control_pair_to_hex(double axial_vel, double turning_rate)
{
    const TelemFrame frame = encode_control_pair(axial_vel, turning_rate);
    return frame_to_hex(frame);
}

}  // namespace telem_binary
