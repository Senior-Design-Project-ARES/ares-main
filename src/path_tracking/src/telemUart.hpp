#pragma once

#include <string>

namespace telem_uart
{

constexpr const char kDefaultDevice[] = "/dev/ttyTHS1";
constexpr unsigned long kDefaultBaud = 115200;

/// On failure returns -1. If \p err_out is non-null, it receives a short English reason (for logging).
int open_serial(const std::string &path, unsigned long baud, std::string *err_out = nullptr);
bool write_all(int fd, const void *data, size_t len);

}  // namespace telem_uart
