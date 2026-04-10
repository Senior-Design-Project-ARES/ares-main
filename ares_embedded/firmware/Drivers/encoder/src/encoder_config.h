#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include "stm32h7xx_hal.h"

#define ENCODER_PULSES_PER_REV  64U
#define ENCODER_COUNTS_PER_REV  (ENCODER_PULSES_PER_REV * 4U)

#define ENCODER_COUNT 4U

typedef struct
{
	TIM_TypeDef  *tim;
	GPIO_TypeDef *port_a;
	uint32_t      pin_a;
	GPIO_TypeDef *port_b;
	uint32_t      pin_b;
	uint32_t      af;
	uint32_t      rcc_tim_clk;
} encoder_hw_config_t;

/*
 * Upper bits of rcc_tim_clk encode the APB bus; lower bits hold the enable mask.
 * 0 = APB1L, 1 = APB2.
 */
#define ENCODER_RCC_BUS_SHIFT 30U
#define ENCODER_RCC_BUS_MASK  (0x3UL << ENCODER_RCC_BUS_SHIFT)
#define ENCODER_RCC_BUS_APB1L 0U
#define ENCODER_RCC_BUS_APB2  1U
#define ENCODER_RCC_ENCODE(bus, mask) ((((uint32_t)(bus)) << ENCODER_RCC_BUS_SHIFT) | (mask))
#define ENCODER_RCC_GET_BUS(val)      (((val) & ENCODER_RCC_BUS_MASK) >> ENCODER_RCC_BUS_SHIFT)
#define ENCODER_RCC_GET_MASK(val)     ((val) & ~ENCODER_RCC_BUS_MASK)

static const encoder_hw_config_t g_encoder_hw[ENCODER_COUNT] = {
	/* M1 - TIM3, PC6/PC7, AF2 - CN10 pin 1 (A), pin 11 (B) */
	{
		TIM3,
		GPIOC,
		GPIO_PIN_6,
		GPIOC,
		GPIO_PIN_7,
		GPIO_AF2_TIM3,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM3EN)
	},
	/* M2 - TIM1, PE9/PE11, AF1 - CN10 pin 4 (A), pin 6 (B) */
	{
		TIM1,
		GPIOE,
		GPIO_PIN_9,
		GPIOE,
		GPIO_PIN_11,
		GPIO_AF1_TIM1,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB2, RCC_APB2ENR_TIM1EN)
	},
	/* M3 - TIM4, PB6/PB7, AF2 - CN10 pin 14 (A), pin 16 (B) */
	{
		TIM4,
		GPIOB,
		GPIO_PIN_6,
		GPIOB,
		GPIO_PIN_7,
		GPIO_AF2_TIM4,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM4EN)
	},
	/* M4 - TIM5, PA0/PA1, AF2 - CN11 pin 28 (A), pin 30 (B) [morpho header] */
	{
		TIM5,
		GPIOA,
		GPIO_PIN_0,
		GPIOA,
		GPIO_PIN_1,
		GPIO_AF2_TIM5,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM5EN)
	}
};

#endif /* ENCODER_CONFIG_H */
