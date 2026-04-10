/*
 * Bare-metal Ethernet + LwIP bring-up for firmware/ (NO_SYS=1).
 *
 * This provides a small API surface:
 *   ethernet_driver_init(cfg)
 *   ethernet_driver_poll()
 *
 * Internally:
 *   - init LwIP
 *   - add netif (ethernetif.c)
 *   - optionally start DHCP
 *   - start a minimal TCP echo server
 */

#include "ethernet_driver.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"
#include "netif/ethernet.h"

#include "ethernetif.h"
#include "tcp_echo_server.h"

static struct netif g_netif;
static ethernet_driver_config_t g_cfg;
static uint32_t g_link_check_ms = 0;

static void default_mac(uint8_t mac[6])
{
  /* locally administered unicast MAC */
  mac[0] = 0x02;
  mac[1] = 0x00;
  mac[2] = 0x00;
  mac[3] = 0x00;
  mac[4] = 0x00;
  mac[5] = 0x01;
}

static void mpu_config_eth_descriptors(void)
{
  /* Match the CubeIDE example's intent: make 0x30000000 region safe for ETH DMA. */
  MPU_Region_InitTypeDef r = {0};

  HAL_MPU_Disable();

  r.Enable           = MPU_REGION_ENABLE;
  r.Number           = MPU_REGION_NUMBER0;
  r.BaseAddress      = 0x30000000;
  r.Size             = MPU_REGION_SIZE_32KB;
  r.SubRegionDisable = 0x00;
  r.TypeExtField     = MPU_TEX_LEVEL0;
  r.AccessPermission = MPU_REGION_FULL_ACCESS;
  r.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  r.IsShareable      = MPU_ACCESS_SHAREABLE;
  r.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  r.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&r);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void ethernet_driver_init(const ethernet_driver_config_t *cfg)
{
  memset(&g_cfg, 0, sizeof(g_cfg));
  if (cfg)
    g_cfg = *cfg;

  if (g_cfg.tcp_port == 0)
    g_cfg.tcp_port = 7;

  if ((g_cfg.mac[0] | g_cfg.mac[1] | g_cfg.mac[2] | g_cfg.mac[3] | g_cfg.mac[4] | g_cfg.mac[5]) == 0)
    default_mac(g_cfg.mac);

  /* Ensure D2 SRAM1 clock is enabled (ETH DMA descriptors live at 0x30000000). */
  __HAL_RCC_D2SRAM1_CLK_ENABLE();

  mpu_config_eth_descriptors();

  lwip_init();

  ip4_addr_t ipaddr, netmask, gw;
  if (g_cfg.use_dhcp)
  {
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);
    ip4_addr_set_zero(&gw);
  }
  else
  {
    IP4_ADDR(&ipaddr,  g_cfg.ip[0],      g_cfg.ip[1],      g_cfg.ip[2],      g_cfg.ip[3]);
    IP4_ADDR(&netmask, g_cfg.netmask[0], g_cfg.netmask[1], g_cfg.netmask[2], g_cfg.netmask[3]);
    IP4_ADDR(&gw,      g_cfg.gw[0],      g_cfg.gw[1],      g_cfg.gw[2],      g_cfg.gw[3]);
  }

  netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, ethernet_input);
  netif_set_default(&g_netif);

  /* Bring link status up/down based on PHY state. */
  ethernet_link_check_state(&g_netif);

  if (g_cfg.use_dhcp)
    (void)dhcp_start(&g_netif);

  tcp_echo_server_init(g_cfg.tcp_port);
}

void ethernet_driver_poll(void)
{
  ethernetif_input(&g_netif);
  sys_check_timeouts();

  /* Link check every 100ms */
  const uint32_t now = HAL_GetTick();
  if ((now - g_link_check_ms) >= 100U)
  {
    g_link_check_ms = now;
    ethernet_link_check_state(&g_netif);
  }
}

