/**
 * @file eth.cpp
 * @brief test ethernet connectivity
 * @author Josh Colgrove
 *
 * Host-side Ethernet connectivity test for the STM32 TCP echo server.
 *
 * Assumes the STM32 firmware is running an echo server (default port 7).
 *
 * Build (macOS/Linux):
 *   g++ -std=c++17 -O2 -Wall -Wextra -pedantic test_eth.cpp -o test_eth
 *
 * Run:
 *   ./test_eth <ip> [port] [message]
 *
 * Examples:
 *   ./test_eth 192.168.0.10
 *   ./test_eth 192.168.0.10 7 "hello stm32"
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

static int set_recv_timeout_ms(int fd, int timeout_ms)
{
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <ip> [port] [message]\n";
    return 2;
  }

  const std::string ip = argv[1];
  const uint16_t port = (argc >= 3) ? static_cast<uint16_t>(std::stoi(argv[2])) : 7;
  const std::string msg = (argc >= 4) ? argv[3] : "ping";

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    std::cerr << "socket() failed: " << std::strerror(errno) << "\n";
    return 1;
  }

  // Keep the test snappy: fail fast if nothing comes back.
  (void)set_recv_timeout_ms(fd, 1500);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
  {
    std::cerr << "inet_pton() failed for ip='" << ip << "'\n";
    ::close(fd);
    return 2;
  }

  std::cout << "Connecting to " << ip << ":" << port << "...\n";
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
  {
    std::cerr << "connect() failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return 1;
  }

  std::cout << "Connected. Sending " << msg.size() << " bytes...\n";
  const ssize_t sent = ::send(fd, msg.data(), msg.size(), 0);
  if (sent < 0)
  {
    std::cerr << "send() failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return 1;
  }

  std::string rx;
  rx.resize(static_cast<size_t>(sent));

  ssize_t got_total = 0;
  while (got_total < sent)
  {
    const ssize_t got = ::recv(fd, rx.data() + got_total, static_cast<size_t>(sent - got_total), 0);
    if (got == 0)
    {
      std::cerr << "recv(): peer closed connection early\n";
      ::close(fd);
      return 1;
    }
    if (got < 0)
    {
      std::cerr << "recv() failed: " << std::strerror(errno) << "\n";
      ::close(fd);
      return 1;
    }
    got_total += got;
  }

  ::close(fd);

  const bool ok = (rx == msg.substr(0, static_cast<size_t>(sent)));
  if (!ok)
  {
    std::cerr << "FAIL: echo mismatch\n";
    std::cerr << "  sent: '" << msg.substr(0, static_cast<size_t>(sent)) << "'\n";
    std::cerr << "  recv: '" << rx << "'\n";
    return 1;
  }

  std::cout << "PASS: received exact echo: '" << rx << "'\n";
  return 0;
}

