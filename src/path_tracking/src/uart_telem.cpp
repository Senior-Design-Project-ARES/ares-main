// Minimal UART helper for Jetson (Linux termios). Read/write raw bytes on a serial device.
//
// Jetson Orin Nano — J12 40-pin header, UART1 (this tool defaults to the matching device node):
//   /dev/ttyTHS0  ← UART1 on the header
//   Pin 8  = UART1_TX  (Jetson TX → connect to STM32 RX)
//   Pin 10 = UART1_RX  (Jetson RX ← connect to STM32 TX)
//   GND: pin 6, 9, 14, 20, 25, 30, 34, or 39 (common ground with STM32)
// Optional hardware flow control (not enabled in termios here; 3-wire link is default):
//   Pin 11 = UART1_RTS, Pin 36 = UART1_CTS
//
// Examples:
//   ros2 run path_tracking uart_telem
//   ros2 run path_tracking uart_telem 115200
//   ros2 run path_tracking uart_telem /dev/ttyTHS0 115200
//   ros2 run path_tracking telem_encode_binary encode | ros2 run path_tracking uart_telem --tx

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{

// UART1 on the Orin Nano J12 header is exposed as this device (pins 8 TX / 10 RX).
constexpr const char kJetsonOrinNanoJ12Uart1Dev[] = "/dev/ttyTHS0";

bool all_digits(const std::string &s)
{
    if (s.empty())
    {
        return false;
    }
    for (char c : s)
    {
        if (!std::isdigit(static_cast<unsigned char>(c)))
        {
            return false;
        }
    }
    return true;
}

speed_t baud_to_constant(unsigned long baud)
{
    switch (baud)
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            return B0;
    }
}

int open_serial(const std::string &path, unsigned long baud)
{
    const int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "open " << path << ": " << std::strerror(errno) << '\n';
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "tcgetattr: " << std::strerror(errno) << '\n';
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
#if defined(CMSPAR)
    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CMSPAR));
#else
    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB));
#endif

    const speed_t spd = baud_to_constant(baud);
    if (spd == B0)
    {
        std::cerr << "unsupported baud " << baud << '\n';
        close(fd);
        return -1;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "tcsetattr: " << std::strerror(errno) << '\n';
        close(fd);
        return -1;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    return fd;
}

void usage(const char *argv0)
{
    std::cerr << "Usage:\n"
              << "  " << argv0 << " [device] [baud] [--tx] [--hex]\n"
              << "  " << argv0 << " [baud] [--tx] [--hex]   (device defaults to " << kJetsonOrinNanoJ12Uart1Dev
              << ")\n"
              << "Modes:\n"
              << "  (default) read from UART → stdout (binary)\n"
              << "  --tx      read stdin → UART\n"
              << "  --hex     read from UART → hex lines on stderr\n"
              << "Defaults: device " << kJetsonOrinNanoJ12Uart1Dev
              << " (Orin Nano J12 UART1: pin 8 TX, pin 10 RX), baud 115200\n";
}

int pump_tx(int fd)
{
    std::vector<char> buf(4096);
    for (;;)
    {
        std::cin.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const std::streamsize n = std::cin.gcount();
        if (n <= 0)
        {
            break;
        }
        const char *p = buf.data();
        std::streamsize left = n;
        while (left > 0)
        {
            const ssize_t w = write(fd, p, static_cast<size_t>(left));
            if (w < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                std::cerr << "write: " << std::strerror(errno) << '\n';
                return 1;
            }
            p += w;
            left -= w;
        }
        if (std::cin.eof())
        {
            break;
        }
    }
    return 0;
}

int pump_rx(int fd, bool hex_dump)
{
    std::vector<uint8_t> buf(4096);
    for (;;)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int maxfd = fd;
        timeval tv{};
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        const int r = select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
        if (r < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            std::cerr << "select: " << std::strerror(errno) << '\n';
            return 1;
        }
        if (r == 0)
        {
            continue;
        }
        const ssize_t n = read(fd, buf.data(), buf.size());
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            std::cerr << "read: " << std::strerror(errno) << '\n';
            return 1;
        }
        if (n == 0)
        {
            continue;
        }
        if (hex_dump)
        {
            for (ssize_t i = 0; i < n; ++i)
            {
                char line[4];
                std::snprintf(line, sizeof(line), "%02x ", static_cast<unsigned int>(buf[static_cast<size_t>(i)]));
                std::cerr << line;
            }
            std::cerr << '\n';
        }
        else
        {
            const char *p = reinterpret_cast<const char *>(buf.data());
            std::cout.write(p, n);
            std::cout.flush();
        }
    }
}

}  // namespace

int main(int argc, char **argv)
{
    bool tx_mode = false;
    bool hex_dump = false;
    std::vector<std::string> pos;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help")
        {
            usage(argv[0]);
            return 0;
        }
        if (a == "--tx")
        {
            tx_mode = true;
            continue;
        }
        if (a == "--hex")
        {
            hex_dump = true;
            continue;
        }
        pos.push_back(a);
    }

    std::string dev = kJetsonOrinNanoJ12Uart1Dev;
    unsigned long baud = 115200; // this is what the stm32 likes (tested by vish)

    if (pos.size() > 2U)
    {
        std::cerr << "too many arguments\n";
        usage(argv[0]);
        return 2;
    }
    if (pos.size() == 1U)
    {
        const std::string &a = pos[0];
        if (all_digits(a))
        {
            try
            {
                baud = std::stoul(a);
            }
            catch (...)
            {
                std::cerr << "bad baud: " << a << '\n';
                return 2;
            }
        }
        else
        {
            dev = a;
        }
    }
    else if (pos.size() == 2U)
    {
        dev = pos[0];
        try
        {
            baud = std::stoul(pos[1]);
        }
        catch (...)
        {
            std::cerr << "bad baud: " << pos[1] << '\n';
            return 2;
        }
    }

    const int fd = open_serial(dev, baud);
    if (fd < 0)
    {
        return 1;
    }

    if (tx_mode)
    {
        const int rc = pump_tx(fd);
        close(fd);
        return rc;
    }

    const int rc = pump_rx(fd, hex_dump);
    close(fd);
    return rc;
}
