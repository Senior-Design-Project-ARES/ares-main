#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                     1
#define LWIP_SOCKET                0
#define LWIP_NETCONN               0

#define SYS_LIGHTWEIGHT_PROT       0

#define MEM_ALIGNMENT              4
#define MEM_SIZE                   16000

#define PBUF_POOL_SIZE             16
#define PBUF_POOL_BUFSIZE          1524

#define LWIP_IPV4                  1
#define LWIP_ARP                   1
#define LWIP_ICMP                  1
#define LWIP_DHCP                  0

#define LWIP_UDP                   1
#define LWIP_TCP                   1

#define TCP_MSS                    1460
#define TCP_SND_BUF                (4 * TCP_MSS)
#define TCP_WND                    (4 * TCP_MSS)

#endif
