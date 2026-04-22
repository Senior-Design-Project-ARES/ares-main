/**
 * @file uart_echo.cpp
 * @brief Minimal raw UART RX/echo test for ST-LINK VCP path validation.
 *
 * Behavior:
 * - Polls uart_try_read_byte() continuously
 * - Echoes each received byte back to host
 * - Toggles LD1 on each RX hit
 * - Prints poll/rx counters once per second
 */

#include <cstdint>

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "uart_driver.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

namespace {

void led_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin   = GPIO_PIN_0;  // LD1 green
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

}  // namespace

int main(void)
{
    HAL_Init();
    uart_driver_init(&uart_driver_config_stlink_vcp);
    led_init();

    println("uart_echo ready (ST-LINK VCP / USART3 PD8-PD9 @ 115200)");
    println("Send bytes from host; each byte will be echoed.");

    uint32_t poll_calls   = 0U;
    uint32_t rx_hits      = 0U;
    uint32_t last_stat_ms = HAL_GetTick();

    while (true) {
        ++poll_calls;
        uint8_t b = 0U;
        if (uart_try_read_byte(&b)) {
            ++rx_hits;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
            // Echo one byte back to host.
            print("%c", static_cast<char>(b));
        }

        const uint32_t now_ms = HAL_GetTick();
        if ((now_ms - last_stat_ms) >= 1000U) {
            println("STAT poll_calls=%lu rx_hits=%lu",
                    static_cast<unsigned long>(poll_calls),
                    static_cast<unsigned long>(rx_hits));
            last_stat_ms = now_ms;
        }
    }
}

extern "C" {
void _init(void) {}
}

