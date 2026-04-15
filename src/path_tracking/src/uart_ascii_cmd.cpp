#include "uart_ascii_cmd.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace uart_ascii_cmd {
namespace {

bool serial_set_baud(termios &tty, unsigned long baud)
{
    speed_t speed = B0;
    switch (baud)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
#ifdef B460800
        case 460800: speed = B460800; break;
#endif
#ifdef B921600
        case 921600: speed = B921600; break;
#endif
        default: return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    return true;
}

} // namespace

/** Raw 8N1 serial, matching tests/uart_over_usb.py open_serial(). */
int open_serial_ascii(const std::string &path, unsigned long baud, std::string *err_out)
{
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        if (err_out != nullptr)
        {
            *err_out = std::strerror(errno);
        }
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        if (err_out != nullptr)
        {
            *err_out = std::strerror(errno);
        }
        close(fd);
        return -1;
    }

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag = static_cast<tcflag_t>(CREAD | CLOCAL | CS8);
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (!serial_set_baud(tty, baud))
    {
        if (err_out != nullptr)
        {
            *err_out = "unsupported uart_baud";
        }
        close(fd);
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        if (err_out != nullptr)
        {
            *err_out = std::strerror(errno);
        }
        close(fd);
        return -1;
    }

    return fd;
}

bool write_all_fd(int fd, const void *buf, size_t len)
{
    const auto *p = static_cast<const char *>(buf);
    size_t off = 0;
    while (off < len)
    {
        const ssize_t n = write(fd, p + off, len - off);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return false;
        }
        off += static_cast<size_t>(n);
    }
    return true;
}

int format_ascii_cmd(char *out, size_t out_sz, double vx, double yaw_rate)
{
    return std::snprintf(out, out_sz, "V%.3f,Y%.3f\n", vx, yaw_rate);
}

} // namespace uart_ascii_cmd
