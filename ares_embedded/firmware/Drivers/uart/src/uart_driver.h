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

/**
 * Board default: USART3, PD8/PD9, 115200 8N1 (copy and override fields for
 * custom pins while keeping the same peripheral/baud if desired).
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

/**
 * Non-blocking receive (same semantics as the former `uart_try_read_byte`).
 *
 * @return 1 if a byte was stored in @p out_byte, else 0.
 */
uint8_t uart_try_read_byte(uint8_t *out_byte);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */
