/*
 * lwipopts.h
 *
 * LwIP configuration for bare-metal (NO_SYS) STM32H723.
 *
 * Keep this minimal; enable more features as needed.
 */

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* ---------- Core options ---------- */
#define NO_SYS                         1
#define LWIP_TIMERS                    1

#define LWIP_IPV4                      1
#define LWIP_IPV6                      0

#define LWIP_ETHERNET                  1

/* ---------- Memory options ---------- */
#define MEM_ALIGNMENT                  4
#define MEM_SIZE                       (16 * 1024)

#define MEMP_NUM_PBUF                  16
#define PBUF_POOL_SIZE                 16
#define PBUF_POOL_BUFSIZE              1536

/* ---------- Protocol options ---------- */
#define LWIP_ARP                       1
#define LWIP_ICMP                      1
#define LWIP_UDP                       1
#define LWIP_TCP                       1
#define TCP_TTL                        255
#define TCP_MSS                        1460
#define TCP_SND_BUF                    (4 * TCP_MSS)
#define TCP_WND                        (4 * TCP_MSS)

/* Raw API only (no netconn / sockets) */
#define LWIP_NETCONN                   0
#define LWIP_SOCKET                    0

/* DHCP optional (driver supports it if enabled) */
#define LWIP_DHCP                      1

/* ---------- Checksums ---------- */
#define CHECKSUM_GEN_IP                1
#define CHECKSUM_GEN_UDP               1
#define CHECKSUM_GEN_TCP               1
#define CHECKSUM_CHECK_IP              1
#define CHECKSUM_CHECK_UDP             1
#define CHECKSUM_CHECK_TCP             1

/* ---------- Debug ---------- */
#define LWIP_DEBUG                     0

#endif /* LWIPOPTS_H */

