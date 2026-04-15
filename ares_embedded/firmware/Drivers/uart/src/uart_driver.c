#include "uart_driver.h"

#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"

static UART_HandleTypeDef g_huart;
static uint8_t            g_uart_initialised = 0U;

const uart_driver_config_t uart_driver_config_debug_default = {
    .instance  = USART3,
    .tx_port   = GPIOD,
    .tx_pin    = GPIO_PIN_8,
    .rx_port   = GPIOD,
    .rx_pin    = GPIO_PIN_9,
    .alternate = GPIO_AF7_USART3,
    .baud_rate = 115200U,
};

static void uart_rcc_gpio_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#if defined(GPIOI)
    else if (port == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
#endif
    else if (port == GPIOJ)
    {
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    }
    else if (port == GPIOK)
    {
        __HAL_RCC_GPIOK_CLK_ENABLE();
    }
}

static uint8_t uart_rcc_usart_enable(USART_TypeDef *inst)
{
    if (inst == NULL)
    {
        return 0U;
    }
    if (inst == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
    }
    else if (inst == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
    }
    else if (inst == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
    }
    else if (inst == UART4)
    {
        __HAL_RCC_UART4_CLK_ENABLE();
    }
    else if (inst == UART5)
    {
        __HAL_RCC_UART5_CLK_ENABLE();
    }
    else if (inst == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();
    }
    else if (inst == UART7)
    {
        __HAL_RCC_UART7_CLK_ENABLE();
    }
    else if (inst == UART8)
    {
        __HAL_RCC_UART8_CLK_ENABLE();
    }
    else if (inst == UART9)
    {
        __HAL_RCC_UART9_CLK_ENABLE();
    }
    else if (inst == USART10)
    {
        __HAL_RCC_USART10_CLK_ENABLE();
    }
    else if (inst == LPUART1)
    {
        __HAL_RCC_LPUART1_CLK_ENABLE();
    }
    else
    {
        return 0U;
    }
    return 1U;
}

static void uart_gpio_af_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t alternate)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = alternate;
    HAL_GPIO_Init(port, &gpio);
}

void uart_driver_init(const uart_driver_config_t *cfg)
{
    if (g_uart_initialised)
    {
        return;
    }

    const uart_driver_config_t *c = (cfg != NULL) ? cfg : &uart_driver_config_debug_default;

    if (c->instance == NULL || c->tx_port == NULL || c->rx_port == NULL || c->tx_pin == 0U
        || c->rx_pin == 0U || c->baud_rate == 0U)
    {
        return;
    }

    uart_rcc_gpio_enable(c->tx_port);
    if (c->rx_port != c->tx_port)
    {
        uart_rcc_gpio_enable(c->rx_port);
    }

    if (!uart_rcc_usart_enable(c->instance))
    {
        return;
    }

    uart_gpio_af_pin(c->tx_port, c->tx_pin, c->alternate);
    uart_gpio_af_pin(c->rx_port, c->rx_pin, c->alternate);

    memset(&g_huart, 0, sizeof(g_huart));
    g_huart.Instance                   = c->instance;
    g_huart.Init.BaudRate              = c->baud_rate;
    g_huart.Init.WordLength            = UART_WORDLENGTH_8B;
    g_huart.Init.StopBits              = UART_STOPBITS_1;
    g_huart.Init.Parity                = UART_PARITY_NONE;
    g_huart.Init.Mode                  = UART_MODE_TX_RX;
    g_huart.Init.HwFlowCtl             = UART_HWCONTROL_NONE;
    g_huart.Init.OverSampling          = UART_OVERSAMPLING_16;
    g_huart.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
    g_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&g_huart) == HAL_OK)
    {
        g_uart_initialised = 1U;
    }
}

void uart_driver_init_default(void)
{
    uart_driver_init(NULL);
}

static void uart_vprint(const char *fmt, va_list args, uint8_t add_newline)
{
    if (!g_uart_initialised || fmt == NULL)
    {
        return;
    }

    char buffer[128];
    int  len = vsnprintf(buffer, (int)sizeof(buffer), fmt, args);

    if (len <= 0)
    {
        return;
    }
    if (len > (int)sizeof(buffer))
    {
        len = (int)sizeof(buffer);
    }

    if (add_newline)
    {
        if (len + 2 <= (int)sizeof(buffer))
        {
            buffer[len++] = '\r';
            buffer[len++] = '\n';
        }
    }

    (void)HAL_UART_Transmit(&g_huart, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

void print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    uart_vprint(fmt, args, 0U);
    va_end(args);
}

void println(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    uart_vprint(fmt, args, 1U);
    va_end(args);
}

uint8_t uart_try_read_byte(uint8_t *out_byte)
{
    if (!g_uart_initialised || out_byte == NULL)
    {
        return 0U;
    }

    HAL_StatusTypeDef st = HAL_UART_Receive(&g_huart, out_byte, 1U, 0U);
    return (st == HAL_OK) ? 1U : 0U;
}

int __io_putchar(int ch)
{
    if (!g_uart_initialised)
    {
        return ch;
    }
    uint8_t c = (uint8_t)ch;
    (void)HAL_UART_Transmit(&g_huart, &c, 1U, HAL_MAX_DELAY);
    return ch;
}

int __io_getchar(void)
{
    uint8_t c = 0U;
    if (!g_uart_initialised)
    {
        return -1;
    }
    (void)HAL_UART_Receive(&g_huart, &c, 1U, HAL_MAX_DELAY);
    return (int)c;
}
