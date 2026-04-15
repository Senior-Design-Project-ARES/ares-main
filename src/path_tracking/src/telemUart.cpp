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


#include "telemUart.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

namespace telem_uart
{

namespace
{

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

}  // namespace

int open_serial(const std::string &path, unsigned long baud, std::string *err_out)
{
    const auto set_err = [err_out](const std::string &msg) {
        if (err_out != nullptr)
        {
            *err_out = msg;
        }
        std::cerr << msg << '\n';
    };

    const int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        set_err("open " + path + ": " + std::strerror(errno));
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        set_err("tcgetattr " + path + ": " + std::strerror(errno));
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
        set_err("unsupported baud " + std::to_string(baud) + " for " + path);
        close(fd);
        return -1;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        set_err("tcsetattr " + path + ": " + std::strerror(errno));
        close(fd);
        return -1;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    return fd;
}

bool write_all(int fd, const void *data, size_t len)
{
    const auto *p = static_cast<const uint8_t *>(data);
    size_t left = len;
    while (left > 0U)
    {
        const ssize_t n = write(fd, p, left);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            std::cerr << "write: " << std::strerror(errno) << '\n';
            return false;
        }
        p += static_cast<size_t>(n);
        left -= static_cast<size_t>(n);
    }
    return true;
}

}  // namespace telem_uart
