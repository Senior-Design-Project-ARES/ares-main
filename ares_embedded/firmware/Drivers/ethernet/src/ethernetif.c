/*
 * Ethernet netif glue for LwIP (STM32H7 HAL ETH, RMII, LAN8742 PHY).
 *
 * This is adapted from ST's LwIP TCP Echo Server example and trimmed to the
 * pieces needed for a bare-metal polling (NO_SYS) integration.
 *
 * Key requirement:
 *   DMA descriptors + RX pool live in SRAM that ETH DMA can access.
 *   The firmware linker script must provide the sections used here:
 *     .RxDescripSection, .TxDescripSection, .Rx_PoolSection at 0x30000000...
 */

#include "ethernetif.h"

#include <string.h>
#include <stddef.h>

#include "stm32h7xx_hal.h"

#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"

#include "lan8742.h"

/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_DMA_TRANSMIT_TIMEOUT (20U)

#define ETH_RX_BUFFER_SIZE 1000U
#define ETH_RX_BUFFER_CNT  12U

typedef enum
{
  RX_ALLOC_OK    = 0x00,
  RX_ALLOC_ERROR = 0x01
} RxAllocStatusTypeDef;

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUFFER_SIZE + 31) & ~31] __ALIGNED(32);
} RxBuff_t;

/* DMA descriptor placement (see linker script) */
#if defined(__GNUC__) || defined(__ARMCC_VERSION)
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDescripSection")));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDescripSection")));
#else
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];
#endif

LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

#if defined(__GNUC__) || defined(__ARMCC_VERSION)
/* Move LwIP RX pool backing store into D2 SRAM (see linker script). */
__attribute__((section(".Rx_PoolSection"))) extern u8_t memp_memory_RX_POOL_base[];
#endif

static RxAllocStatusTypeDef RxAllocStatus = RX_ALLOC_OK;

/* Global Ethernet handle */
ETH_HandleTypeDef EthHandle;

/* PHY */
static lan8742_Object_t LAN8742;
static int32_t ETH_PHY_IO_Init(void);
static int32_t ETH_PHY_IO_DeInit(void);
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
static int32_t ETH_PHY_IO_GetTick(void);

static lan8742_IOCtx_t LAN8742_IOCtx = {
    ETH_PHY_IO_Init,
    ETH_PHY_IO_DeInit,
    ETH_PHY_IO_WriteReg,
    ETH_PHY_IO_ReadReg,
    ETH_PHY_IO_GetTick,
};

static void pbuf_free_custom(struct pbuf *p);

static void low_level_init(struct netif *netif)
{
  HAL_StatusTypeDef st;

  /* Use MAC from HAL conf by default */
  static uint8_t macaddress[6] = {
      ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

  st = HAL_ETH_Init(&EthHandle);
  if (st != HAL_OK)
  {
    netif_set_down(netif);
    netif_set_link_down(netif);
    return;
  }

  netif->hwaddr_len = ETH_HWADDR_LEN;
  memcpy(netif->hwaddr, macaddress, 6);

  netif->mtu = ETH_MAX_PAYLOAD;
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  LWIP_MEMPOOL_INIT(RX_POOL);

  (void)LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
  if (LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
  {
    netif_set_down(netif);
    netif_set_link_down(netif);
    return;
  }

  ethernet_link_check_state(netif);
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  (void)netif;

  uint32_t i = 0U;
  struct pbuf *q;

  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};
  ETH_TxPacketConfigTypeDef TxConfig;
  memset(&TxConfig, 0, sizeof(TxConfig));

  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  for (q = p; q != NULL; q = q->next)
  {
    if (i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer = (uint8_t *)q->payload;
    Txbuffer[i].len = q->len;
    Txbuffer[i].next = (i + 1 < ETH_TX_DESC_CNT) ? &Txbuffer[i + 1] : NULL;

    if (q->next == NULL)
      Txbuffer[i].next = NULL;

    i++;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;
  TxConfig.pData = p;

  if (HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT) != HAL_OK)
    return ERR_IF;

  return ERR_OK;
}

static struct pbuf *low_level_input(struct netif *netif)
{
  (void)netif;

  struct pbuf *p = NULL;
  if (RxAllocStatus == RX_ALLOC_OK)
  {
    HAL_ETH_ReadData(&EthHandle, (void **)&p);
  }
  return p;
}

void ethernetif_input(struct netif *netif)
{
  struct pbuf *p;
  do
  {
    p = low_level_input(netif);
    if (p != NULL)
    {
      if (netif->input(p, netif) != ERR_OK)
        pbuf_free(p);
    }
  } while (p != NULL);
}

err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

#if LWIP_IPV4 && (LWIP_ARP || LWIP_ETHERNET)
  netif->output = etharp_output;
#endif
  netif->linkoutput = low_level_output;

  low_level_init(netif);
  return ERR_OK;
}

/* --- PHY / link management --- */

void ethernet_link_check_state(struct netif *netif)
{
  int32_t link_state = LAN8742_GetLinkState(&LAN8742);

  if (link_state < LAN8742_STATUS_OK)
  {
    netif_set_link_down(netif);
    netif_set_down(netif);
    return;
  }

  if (!netif_is_link_up(netif))
  {
    uint32_t speed = ETH_SPEED_100M;
    uint32_t duplex = ETH_FULLDUPLEX_MODE;

    if (link_state == LAN8742_STATUS_100MBITS_FULLDUPLEX)
    {
      speed = ETH_SPEED_100M;
      duplex = ETH_FULLDUPLEX_MODE;
    }
    else if (link_state == LAN8742_STATUS_100MBITS_HALFDUPLEX)
    {
      speed = ETH_SPEED_100M;
      duplex = ETH_HALFDUPLEX_MODE;
    }
    else if (link_state == LAN8742_STATUS_10MBITS_FULLDUPLEX)
    {
      speed = ETH_SPEED_10M;
      duplex = ETH_FULLDUPLEX_MODE;
    }
    else
    {
      speed = ETH_SPEED_10M;
      duplex = ETH_HALFDUPLEX_MODE;
    }

    ETH_MACConfigTypeDef macconf;
    HAL_ETH_GetMACConfig(&EthHandle, &macconf);
    macconf.DuplexMode = duplex;
    macconf.Speed = speed;
    (void)HAL_ETH_SetMACConfig(&EthHandle, &macconf);

    (void)HAL_ETH_Start(&EthHandle);
    netif_set_up(netif);
    netif_set_link_up(netif);
  }
}

/* --- RX pool hooks expected by HAL ETH --- */

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  RxBuff_t *p = (RxBuff_t *)LWIP_MEMPOOL_ALLOC(RX_POOL);
  if (p)
  {
    *buff = (uint8_t *)p->buff;
    p->pbuf_custom.custom_free_function = pbuf_free_custom;
    RxAllocStatus = RX_ALLOC_OK;
  }
  else
  {
    *buff = NULL;
    RxAllocStatus = RX_ALLOC_ERROR;
  }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  (void)pEnd;
  struct pbuf_custom *custom = (struct pbuf_custom *)((uint32_t)buff - offsetof(RxBuff_t, buff));

  custom->pbuf.next = NULL;
  custom->pbuf.payload = buff;
  custom->pbuf.len = Length;
  custom->pbuf.tot_len = Length;
  custom->pbuf.type_internal = PBUF_TYPE_ALLOC_SRC_MASK | PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS;
  custom->pbuf.flags = 0;
  custom->pbuf.ref = 1;

  *pStart = &custom->pbuf;
}

static void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom *custom_pbuf = (struct pbuf_custom *)p;
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}

/* --- HAL ETH MSP (GPIO + clocks) --- */

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  if (heth->Instance != ETH)
    return;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF11_ETH;

  /* RMII pins (NUCLEO-H723ZG typical mapping) */
  /* PA1: REF_CLK, PA2: MDIO, PA7: CRS_DV */
  g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &g);

  /* PC1: MDC, PC4: RXD0, PC5: RXD1 */
  g.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &g);

  /* PG11: TX_EN, PG13: TXD0, PG14: TXD1 */
  g.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOG, &g);

  HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
}

/* --- PHY IO over ETH MDIO --- */

static int32_t ETH_PHY_IO_Init(void)
{
  return 0;
}

static int32_t ETH_PHY_IO_DeInit(void)
{
  return 0;
}

static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if (HAL_ETH_ReadPHYRegister(&EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK)
    return -1;
  return 0;
}

static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if (HAL_ETH_WritePHYRegister(&EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK)
    return -1;
  return 0;
}

static int32_t ETH_PHY_IO_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

