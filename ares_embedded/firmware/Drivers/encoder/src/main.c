// "pio run -t upload" to flash
// "pio device monitor" to see output

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "encoder_backend.h"
#include "encoder_config.h"

#define RPM_SAMPLE_INTERVAL_MS 10U

static UART_HandleTypeDef huart3;

static void MX_USART3_UART_Init(void);
static void UpdateAndPublishAllRpm(uint32_t now_ms);

void Error_Handler(void);

int main(void)
{
    HAL_Init();
    MX_USART3_UART_Init();
    encoder_backend_init_all();

    uint32_t last_report_ms = HAL_GetTick();
    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_report_ms) >= RPM_SAMPLE_INTERVAL_MS)
        {
            UpdateAndPublishAllRpm(now);
            last_report_ms = now;
        }
    }
}

static void UpdateAndPublishAllRpm(uint32_t now_ms)
{
    static uint32_t prev_ms    = 0U;
    static bool     first_call = true;

    if (first_call)
    {
        first_call = false;
        prev_ms    = now_ms;
        for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
        {
            (void)encoder_backend_get_and_reset_counts(i);
        }
        HAL_UART_Transmit(&huart3, (uint8_t *)"RPM:M1+0.00,M2+0.00,M3+0.00,M4+0.00\r\n", 38, 50);
        return;
    }

    uint32_t elapsed = now_ms - prev_ms;
    if (elapsed == 0U)
    {
        return;
    }

    float rpm[ENCODER_COUNT];
    for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
    {
        int32_t counts = encoder_backend_get_and_reset_counts(i);
        rpm[i] = ((float)counts * 60000.0f) /
                 ((float)ENCODER_COUNTS_PER_REV * (float)elapsed);
    }

    prev_ms = now_ms;

    /* Format: RPM:M1+123.45,M2+0.00,M3+0.00,M4-012.30\r\n */
    static const char *labels[ENCODER_COUNT] = {"M1", "M2", "M3", "M4"};
    char buffer[64];
    int  pos = 0;
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "RPM:");
    for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
    {
        char  sign    = (rpm[i] >= 0.0f) ? '+' : '-';
        float abs_rpm = fabsf(rpm[i]);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "%s%c%.2f%s",
                        labels[i], sign, abs_rpm,
                        (i < ENCODER_COUNT - 1U) ? "," : "\r\n");
    }
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, (uint16_t)pos, 50);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

static void MX_USART3_UART_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct   = {0};
    GPIO_InitStruct.Pin                = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode               = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull               = GPIO_PULLUP;
    GPIO_InitStruct.Speed              = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate          = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

int _write(int file, char *data, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart3, (uint8_t *)data, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        __NOP();
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif