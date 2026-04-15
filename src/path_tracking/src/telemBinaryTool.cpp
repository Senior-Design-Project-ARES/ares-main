#include "telemBinary.hpp"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{

void print_frame(const telem_binary::TelemFrame &f)
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
        telem_binary::TelemFrame f{};
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
        telem_binary::encode_inplace(f);
        std::cout.write(reinterpret_cast<const char *>(&f), sizeof(f));
        return std::cout.good() ? 0 : 1;
    }

    if (mode == "decode")
    {
        std::vector<char> buf(static_cast<size_t>(sizeof(telem_binary::TelemFrame)));
        while (std::cin.read(buf.data(), static_cast<std::streamsize>(buf.size())) || std::cin.gcount() > 0)
        {
            const auto n = static_cast<size_t>(std::cin.gcount());
            if (n != sizeof(telem_binary::TelemFrame))
            {
                std::cerr << "short read (" << n << " bytes), stopping\n";
                return 1;
            }
            telem_binary::TelemFrame f{};
            std::memcpy(&f, buf.data(), sizeof(f));
            if (!telem_binary::decode_check(f))
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
