/**
 * @file uart_driver.h
 * @brief Configurable USART/UART HAL driver (GPIO mux + printf-style I/O).
 *
 * Default: USART3 on PD8 (TX) / PD9 (RX), AF7, 115200 8N1 — matches prior
 * `uart_print` / ST-Link VCP style bring-up.
 *
 * Pin alternate functions must match the STM32H7 reference manual for the
 * chosen peripheral and port (e.g. `GPIO_AF7_USART3`).
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdarg.h>
#include <stdint.h>

/** Hardware mapping for one UART debug / teleop link. */
typedef struct
{
    USART_TypeDef *instance; /**< USART1..USART10, UART4/5/7/8/9, or LPUART1 */
    GPIO_TypeDef *tx_port;
    uint16_t      tx_pin;
    GPIO_TypeDef *rx_port;
    uint16_t      rx_pin;
    uint8_t       alternate; /**< GPIO_AFx_* from `stm32h7xx_hal_gpio_ex.h` */
    uint32_t      baud_rate;
} uart_driver_config_t;

typedef struct
{
    uint32_t irq_rx_bytes;
    uint32_t err_overrun;
    uint32_t err_framing;
    uint32_t err_noise;
    uint32_t err_parity;
    uint32_t ring_overflow;
} uart_driver_rx_stats_t;

/**
 * ST-LINK VCP mapping on NUCLEO-H723ZG: USART3, PD8/PD9, 115200 8N1.
 * This is the USART path used by `uart_try_read_byte` when selected.
 */
extern const uart_driver_config_t uart_driver_config_stlink_vcp;

/**
 * Board default config used by `uart_driver_init_default()`.
 */
extern const uart_driver_config_t uart_driver_config_debug_default;

/**
 * Initialise UART from @p cfg (GPIO + RCC + HAL_UART).
 *
 * @param cfg Mapping and baud, or `NULL` to use @ref uart_driver_config_debug_default.
 *            Safe to call only once before other APIs; later calls are ignored
 *            if already initialised.
 */
void uart_driver_init(const uart_driver_config_t *cfg);

/** Equivalent to `uart_driver_init(NULL)`. */
void uart_driver_init_default(void);

void print(const char *fmt, ...);
void println(const char *fmt, ...);
uint8_t uart_write_bytes(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * Non-blocking receive (same semantics as the former `uart_try_read_byte`).
 *
 * @return 1 if a byte was stored in @p out_byte, else 0.
 */
uint8_t uart_try_read_byte(uint8_t *out_byte);
void uart_driver_irq_handler(void);
void uart_driver_get_rx_stats(uart_driver_rx_stats_t *out_stats);
void uart_driver_reset_rx_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */
