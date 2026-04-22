#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts a simple TCP echo server (raw API). */
void tcp_echo_server_init(uint16_t port);

#ifdef __cplusplus
}
#endif

