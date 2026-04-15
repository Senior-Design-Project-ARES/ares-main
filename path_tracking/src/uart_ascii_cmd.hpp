#pragma once

#include <cstddef>
#include <string>

/** UART ASCII command helpers (matches tests/uart_over_usb.py). */
namespace uart_ascii_cmd {

/** Default CDC/ACM serial device on Linux (USB STM32, etc.). */
constexpr const char kDefaultDevice[] = "/dev/ttyACM0";
constexpr unsigned long kDefaultBaud = 115200;

int open_serial_ascii(const std::string &path, unsigned long baud, std::string *err_out);
bool write_all_fd(int fd, const void *buf, size_t len);

/** Format: V{vx:.3f},Y{yaw:.3f}\\n */
int format_ascii_cmd(char *out, size_t out_sz, double vx, double yaw_rate);

} // namespace uart_ascii_cmd
