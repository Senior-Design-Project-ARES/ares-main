// Binary telemetry frame helpers for Jetson ↔ MCU UART links.
// Frame layout (little-endian, packed):
//   magic u16   0xA5A5
//   ver   u16   schema version
//   seq   u32   monotonic counter
//   t_s   f64   host time (seconds)
//   vx    f32   body x velocity (m/s)
//   vy    f32   body y velocity (m/s)
//   yaw   f32   yaw rate (rad/s)
//   crc   u16   CRC-16-CCITT over bytes [0 .. sizeof-2)

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{

constexpr uint16_t kMagic = 0xA5A5;
constexpr uint16_t kVersion = 1;

#pragma pack(push, 1)
struct TelemFrame
{
    uint16_t magic{kMagic};
    uint16_t version{kVersion};
    uint32_t seq{0};
    double t_s{0.0};
    float vx{0.F};
    float vy{0.F};
    float yaw_rate{0.F};
    uint16_t crc{0};
};
#pragma pack(pop)

static_assert(sizeof(TelemFrame) == 30, "unexpected TelemFrame size");

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

void encode_inplace(TelemFrame &f)
{
    f.magic = kMagic;
    f.version = kVersion;
    f.crc = 0;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&f);
    f.crc = crc16_ccitt(bytes, sizeof(f) - sizeof(f.crc));
}

bool decode_check(const TelemFrame &f)
{
    if (f.magic != kMagic || f.version != kVersion)
    {
        return false;
    }
    TelemFrame copy = f;
    const uint16_t got = copy.crc;
    copy.crc = 0;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&copy);
    const uint16_t expect = crc16_ccitt(bytes, sizeof(copy) - sizeof(copy.crc));
    return got == expect;
}

void print_frame(const TelemFrame &f)
{
    std::cout << "magic=0x" << std::hex << f.magic << std::dec << " ver=" << f.version << " seq=" << f.seq
              << " t_s=" << f.t_s << " vx=" << f.vx << " vy=" << f.vy << " yaw_rate=" << f.yaw_rate << " crc=0x"
              << std::hex << f.crc << std::dec << '\n';
}

void usage(const char *argv0)
{
    std::cerr << "Usage:\n"
              << "  " << argv0 << " encode [seq t_s vx vy yaw_rate]   (defaults: 0 0 0 0 0 0)\n"
              << "      Write one binary frame to stdout.\n"
              << "  " << argv0 << " decode < binary_frames   (or pipe)\n"
              << "      Read frames from stdin; print human-readable lines.\n";
}

}  // namespace

int main(int argc, char **argv)
{
    std::string mode = "encode";
    if (argc >= 2)
    {
        mode = argv[1];
    }

    if (mode == "-h" || mode == "--help")
    {
        usage(argv[0]);
        return 0;
    }

    if (mode == "encode")
    {
        TelemFrame f{};
        if (argc == 2)
        {
            // defaults already zeroed
        }
        else if (argc == 7)
        {
            f.seq = static_cast<uint32_t>(std::stoul(argv[2]));
            f.t_s = std::stod(argv[3]);
            f.vx = std::stof(argv[4]);
            f.vy = std::stof(argv[5]);
            f.yaw_rate = std::stof(argv[6]);
        }
        else
        {
            usage(argv[0]);
            return 2;
        }
        encode_inplace(f);
        std::cout.write(reinterpret_cast<const char *>(&f), sizeof(f));
        return std::cout.good() ? 0 : 1;
    }

    if (mode == "decode")
    {
        std::vector<char> buf(static_cast<size_t>(sizeof(TelemFrame)));
        while (std::cin.read(buf.data(), static_cast<std::streamsize>(buf.size())) || std::cin.gcount() > 0)
        {
            const auto n = static_cast<size_t>(std::cin.gcount());
            if (n != sizeof(TelemFrame))
            {
                std::cerr << "short read (" << n << " bytes), stopping\n";
                return 1;
            }
            TelemFrame f{};
            std::memcpy(&f, buf.data(), sizeof(f));
            if (!decode_check(f))
            {
                std::cerr << "bad frame (magic/ver/crc)\n";
                continue;
            }
            print_frame(f);
        }
        return 0;
    }

    usage(argv[0]);
    return 2;
}
