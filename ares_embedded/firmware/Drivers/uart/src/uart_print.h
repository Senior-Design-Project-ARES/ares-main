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

#ifdef __cplusplus
}
#endif

#endif /* UART_PRINT_H */

