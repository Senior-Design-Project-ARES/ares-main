#include "uart_print.h"

#include <stdio.h>

#include "stm32h7xx_hal.h"

static UART_HandleTypeDef g_huart3;
static uint8_t            g_uart_initialised = 0U;

static void uart_gpio_init(void)
{
    println("GPIO INIT CALLED");
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // TX (PD8)
    GPIO_InitStruct.Pin              = GPIO_PIN_8;
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate        = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // RX (PD9)
    GPIO_InitStruct.Pin              = GPIO_PIN_9;
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate        = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void print_init(void)
{
    uart_gpio_init();

    g_huart3.Instance          = USART3;
    g_huart3.Init.BaudRate     = 9600;
    g_huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    g_huart3.Init.StopBits     = UART_STOPBITS_1;
    g_huart3.Init.Parity       = UART_PARITY_NONE;
    g_huart3.Init.Mode         = UART_MODE_TX_RX;
    g_huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    g_huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    g_huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&g_huart3) == HAL_OK)
    {
        g_uart_initialised = 1U;
    }
    __HAL_UART_ENABLE(&g_huart3);
    println("Print INIT Called");
}

static void uart_vprint(const char *fmt, va_list args, uint8_t add_newline)
{
    if (!g_uart_initialised || fmt == NULL)
    {
        return;
    }

    char    buffer[128];
    int len = vsnprintf(buffer, (int)sizeof(buffer), fmt, args);

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

    HAL_UART_Transmit(&g_huart3, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
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

    HAL_StatusTypeDef st = HAL_UART_Receive(&g_huart3, out_byte, 1U, 0U);
    return (st == HAL_OK) ? 1U : 0U;
}

uint8_t uart_read_byte_blocking(uint8_t *out_byte)
{
    println("Reading started");
    if (!g_uart_initialised || out_byte == NULL)
    {
    	println("reading ended");
        return 0U;
    }

    HAL_UART_Receive(&g_huart3, out_byte, 1U, HAL_MAX_DELAY);
    println("Reading ended");
    return 1U;
}

// Provide __io_putchar for STM32Cube syscalls.c so printf/newlib route here.
int __io_putchar(int ch)
{
    if (!g_uart_initialised)
    {
        return ch;
    }
    uint8_t c = (uint8_t)ch;
    (void)HAL_UART_Transmit(&g_huart3, &c, 1U, HAL_MAX_DELAY);
    return ch;
}

// Optional: simple stub; blocks until a byte is received if ever used.
int __io_getchar(void)
{
    uint8_t c = 0U;
    if (!g_uart_initialised)
    {
        return -1;
    }
    (void)HAL_UART_Receive(&g_huart3, &c, 1U, HAL_MAX_DELAY);
    return (int)c;
}

