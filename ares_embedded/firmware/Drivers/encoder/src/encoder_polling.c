#if !ENCODER_USE_TIM

#include <stdio.h>
#include <string.h>
#include "encoder_backend.h"
#include "encoder_config.h"
#include "stm32h7xx_hal.h"

static volatile int32_t g_counts = 0;
static uint8_t g_last_state = 0;
static volatile uint32_t g_forward_counts = 0;
static volatile uint32_t g_reverse_counts = 0;
static volatile uint32_t g_direction_flips = 0;
static int8_t g_last_delta_sign = 0;

#define TRANSITION_BUFFER_LEN 16U

typedef struct
{
    uint8_t prev_state;
    uint8_t new_state;
    int8_t delta;
} transition_entry_t;

static transition_entry_t g_transition_buffer[TRANSITION_BUFFER_LEN];
static uint8_t g_transition_head = 0;
static uint8_t g_transition_count = 0;

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

void encoder_backend_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef init = {0};
    init.Pin = ENCODER_A_PIN | ENCODER_B_PIN;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENCODER_A_PORT, &init);

    g_last_state = read_state();
    g_counts = 0;
}

void encoder_backend_poll(void)
{
    uint8_t prev_state = g_last_state;
    uint8_t new_state = read_state();
    int8_t delta = kQuadTable[g_last_state][new_state];
    g_last_state = new_state;

    if (delta != 0)
    {
        if (delta > 0)
        {
            g_forward_counts += (uint32_t)delta;
            if (g_last_delta_sign < 0)
            {
                g_direction_flips++;
            }
            g_last_delta_sign = +1;
        }
        else
        {
            g_reverse_counts += (uint32_t)(-delta);
            if (g_last_delta_sign > 0)
            {
                g_direction_flips++;
            }
            g_last_delta_sign = -1;
        }

        g_transition_buffer[g_transition_head].prev_state = prev_state;
        g_transition_buffer[g_transition_head].new_state = new_state;
        g_transition_buffer[g_transition_head].delta = delta;
        g_transition_head = (uint8_t)((g_transition_head + 1U) % TRANSITION_BUFFER_LEN);
        if (g_transition_count < TRANSITION_BUFFER_LEN)
        {
            g_transition_count++;
        }

        g_counts += delta;
    }
}

int32_t encoder_backend_get_and_reset_counts(void)
{
    __disable_irq();
    int32_t counts = g_counts;
    g_counts = 0;
    __enable_irq();
    return counts;
}

void encoder_backend_get_and_reset_direction_stats(encoder_direction_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    __disable_irq();
    stats->forward_counts = g_forward_counts;
    stats->reverse_counts = g_reverse_counts;
    stats->direction_flips = g_direction_flips;
    g_forward_counts = 0;
    g_reverse_counts = 0;
    g_direction_flips = 0;
    __enable_irq();
}

void encoder_backend_dump_transitions(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    transition_entry_t snapshot[TRANSITION_BUFFER_LEN];
    uint8_t count;
    uint8_t head;

    __disable_irq();
    count = g_transition_count;
    head = g_transition_head;
    for (uint8_t i = 0; i < count; ++i)
    {
        uint8_t index = (uint8_t)((head + TRANSITION_BUFFER_LEN - count + i) % TRANSITION_BUFFER_LEN);
        snapshot[i] = g_transition_buffer[index];
    }
    __enable_irq();

    if (count == 0U)
    {
        const char *empty = "T:EMPTY\r\n";
        HAL_UART_Transmit(huart, (const uint8_t *)empty, (uint16_t)strlen(empty), 50);
        return;
    }

    char line[24];
    for (uint8_t i = 0; i < count; ++i)
    {
        int len = snprintf(line, sizeof(line), "T:%u->%u=%+d\r\n",
                           snapshot[i].prev_state,
                           snapshot[i].new_state,
                           snapshot[i].delta);
        if (len > 0)
        {
            HAL_UART_Transmit(huart, (uint8_t *)line, (uint16_t)len, 50);
        }
    }
}

#endif /* ENCODER_USE_POLLING */
