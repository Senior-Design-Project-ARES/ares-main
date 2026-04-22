#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t mac[6];         /* if all zeros, a default locally-administered MAC is used */
  uint8_t use_dhcp;       /* 1 = DHCP, 0 = static */
  uint8_t ip[4];          /* used when use_dhcp = 0 */
  uint8_t netmask[4];     /* used when use_dhcp = 0 */
  uint8_t gw[4];          /* used when use_dhcp = 0 */
  uint16_t tcp_port;      /* echo server port (default 7) */
} ethernet_driver_config_t;

void ethernet_driver_init(const ethernet_driver_config_t *cfg);

/* Call frequently from the main loop (polling, NO_SYS=1) */
void ethernet_driver_poll(void);

#ifdef __cplusplus
}
#endif

