#if !ENCODER_USE_POLLING && !ENCODER_USE_TIM

#include "encoder_backend.h"
#include "encoder_config.h"
#include "stm32h7xx_hal.h"

static volatile int32_t g_counts = 0;
static uint8_t g_last_state = 0;

static const int8_t kQuadTable[4][4] = {
    {0, +1, 0, -1},
    {-1, 0, +1, 0},
    {0, -1, 0, +1},
    {+1, 0, -1, 0},
};

static uint8_t read_state(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(ENCODER_A_PORT, ENCODER_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(ENCODER_B_PORT, ENCODER_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((a << 1) | b);
}

static void process_transition(void)
{
    uint8_t new_state = read_state();
    int8_t delta = kQuadTable[g_last_state][new_state];
    g_last_state = new_state;
    if (delta != 0)
    {
        g_counts += delta;
    }
}

void encoder_backend_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef init = {0};
    init.Pin = ENCODER_A_PIN | ENCODER_B_PIN;
    init.Mode = GPIO_MODE_IT_RISING_FALLING;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENCODER_A_PORT, &init);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    g_last_state = read_state();
    g_counts = 0;
}

void encoder_backend_poll(void)
{
}

int32_t encoder_backend_get_and_reset_counts(void)
{
    __disable_irq();
    int32_t counts = g_counts;
    g_counts = 0;
    __enable_irq();
    return counts;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if ((pin == ENCODER_A_PIN) || (pin == ENCODER_B_PIN))
    {
        process_transition();
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR1 & ((1U << 6) | (1U << 7));
    EXTI->PR1 = pending;
    process_transition();
    /* Sample again in case both channels changed */
    process_transition();
}


void encoder_backend_get_and_reset_direction_stats(encoder_direction_stats_t *stats)
{
    if (stats == NULL) return;
    stats->forward_counts = 0;
    stats->reverse_counts = 0;
    stats->direction_flips = 0;
}

void encoder_backend_dump_transitions(UART_HandleTypeDef *huart)
{
    (void)huart;
}


#endif /* !ENCODER_USE_POLLING && !ENCODER_USE_TIM */
