/**
 * @file uart_print.h/c
 * @brief UART debug print interface for firmware logging.
 * @author Vishnu Duriseti
 */
#ifndef UART_PRINT_H
#define UART_PRINT_H

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise USART3 on PD8/PD9 as a simple TX/RX debug port (115200 8N1).
 * Call once after HAL_Init() and clock configuration.
 */
void print_init(void);

/**
 * Print formatted text over UART without automatically appending a newline.
 * Format string semantics are the same as standard printf.
 */
void print(const char *fmt, ...);

/**
 * Print formatted text over UART and automatically append \"\\r\\n\".
 */
void println(const char *fmt, ...);

/**
 * Try to receive one byte from USART3 without blocking.
 *
 * @param out_byte Destination for received byte (must be non-null).
 * @return 1 if a byte was received, 0 if no data was available.
 */
uint8_t uart_try_read_byte(uint8_t *out_byte);

#ifdef __cplusplus
}
#endif

#endif /* UART_PRINT_H */

