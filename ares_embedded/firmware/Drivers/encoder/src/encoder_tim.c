#if ENCODER_USE_TIM

#include "encoder_backend.h"
#include "encoder_config.h"
#include "stm32h7xx_hal.h"

static uint16_t g_last_counter[ENCODER_COUNT];

static void enable_gpio_clock(GPIO_TypeDef *port)
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
}

static void configure_gpio_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    GPIO_InitTypeDef init = {0};
    init.Pin       = pin;
    init.Mode      = GPIO_MODE_AF_PP;
    init.Pull      = GPIO_PULLUP;
    init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = af;
    HAL_GPIO_Init(port, &init);
}

static void enable_timer_clock(const encoder_hw_config_t *cfg)
{
    uint32_t bus  = ENCODER_RCC_GET_BUS(cfg->rcc_tim_clk);
    uint32_t mask = ENCODER_RCC_GET_MASK(cfg->rcc_tim_clk);

    switch (bus)
    {
        case ENCODER_RCC_BUS_APB1L:
            SET_BIT(RCC->APB1LENR, mask);
            SET_BIT(RCC->APB1LRSTR, mask);
            CLEAR_BIT(RCC->APB1LRSTR, mask);
            break;
        case ENCODER_RCC_BUS_APB2:
            SET_BIT(RCC->APB2ENR, mask);
            SET_BIT(RCC->APB2RSTR, mask);
            CLEAR_BIT(RCC->APB2RSTR, mask);
            break;
        default:
            break;
    }
}

static void configure_timer(const encoder_hw_config_t *cfg)
{
    TIM_TypeDef *tim = cfg->tim;

    tim->CR1   = 0U;
    tim->CR2   = 0U;
    tim->SMCR  = 0U;
    tim->DIER  = 0U;
    tim->CCMR1 = 0U;
    tim->CCMR2 = 0U;
    tim->CCER  = 0U;

    tim->PSC = 0U;
    tim->ARR = 0xFFFFU;

    tim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

    tim->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

    tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

    tim->CNT = 0U;
    tim->CR1 |= TIM_CR1_CEN;
}

static void init_single_encoder(uint8_t id)
{
    if (id >= ENCODER_COUNT)
    {
        return;
    }

    const encoder_hw_config_t *cfg = &g_encoder_hw[id];

    enable_gpio_clock(cfg->port_a);
    enable_gpio_clock(cfg->port_b);

    configure_gpio_pin(cfg->port_a, cfg->pin_a, cfg->af);
    if (cfg->port_b != cfg->port_a || cfg->pin_b != cfg->pin_a)
    {
        configure_gpio_pin(cfg->port_b, cfg->pin_b, cfg->af);
    }

    enable_timer_clock(cfg);
    configure_timer(cfg);
    g_last_counter[id] = 0U;
}

void encoder_backend_init_all(void)
{
    for (uint8_t i = 0U; i < ENCODER_COUNT; ++i)
    {
        init_single_encoder(i);
    }
}

void encoder_backend_poll(uint8_t encoder_id)
{
    (void)encoder_id;
}

int32_t encoder_backend_get_and_reset_counts(uint8_t encoder_id)
{
    if (encoder_id >= ENCODER_COUNT)
    {
        return 0;
    }

    TIM_TypeDef *tim = g_encoder_hw[encoder_id].tim;
    uint16_t current = (uint16_t)tim->CNT;
    int32_t delta = (int32_t)current - (int32_t)g_last_counter[encoder_id];

    if (delta > 32767)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }

    g_last_counter[encoder_id] = current;

    if (delta > -2 && delta < 2)
    {
        delta = 0;
    }

    return delta;
}

#endif /* ENCODER_USE_TIM */
