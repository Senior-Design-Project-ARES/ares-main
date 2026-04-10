#pragma once

#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

err_t ethernetif_init(struct netif *netif);

/* Polling RX path for NO_SYS builds. Call from main loop. */
void ethernetif_input(struct netif *netif);

/* Link state update helper (PHY status -> netif). */
void ethernet_link_check_state(struct netif *netif);

#ifdef __cplusplus
}
#endif

