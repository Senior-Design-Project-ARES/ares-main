#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * 64 CPR (motor shaft): vendor counts per revolution already at quadrature
 * resolution. TIM encoder mode TI12 counts those edges — do not multiply by 4 here.
 */
#define ENCODER_COUNTS_PER_REV 64U

#define ENCODER_COUNT 4U
#define ENCODER_GEAR_RATIO 50U

typedef struct
{
	TIM_TypeDef  *tim;
	GPIO_TypeDef *port_a;
	uint32_t      pin_a;
	GPIO_TypeDef *port_b;
	uint32_t      pin_b;
	uint32_t      af;
	uint32_t      rcc_tim_clk;
	uint32_t      ic_filter; /* IC1F/IC2F [0..15]: 0=no filter, 15=max (reject <2.67 µs at 96 MHz) */
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

/*
 * Per-channel sign applied to raw quadrature deltas before returning them.
 * Use +1 when timer counts increase under that wheel's MOTOR_FORWARD; use -1
 * if the encoder phases are wired so forward motion counts the other way.
 *
 * Index i matches motor_config.h: M1 LR, M2 LF, M3 RR, M4 RF (same as motors[i]).
 * Defined in encoder_tim.c.
 */
extern const int8_t g_encoder_counts_sign_vs_motor_forward[ENCODER_COUNT];

static const encoder_hw_config_t g_encoder_hw[ENCODER_COUNT] = {
	/* M1 LR — TIM3, PC6/PC7, AF2 - CN10 pin 1 (A), pin 11 (B) */
	{
		TIM3,
		GPIOC,
		GPIO_PIN_6,
		GPIOC,
		GPIO_PIN_7,
		GPIO_AF2_TIM3,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM3EN),
		0U
	},
	/* M2 LF — TIM1, PE9/PE11, AF1 - CN10 pin 4 (A), pin 6 (B) */
	{
		TIM1,
		GPIOE,
		GPIO_PIN_9,
		GPIOE,
		GPIO_PIN_11,
		GPIO_AF1_TIM1,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB2, RCC_APB2ENR_TIM1EN),
		0U
	},
	/* M3 RR — TIM4, PB6/PB7, AF2 - CN10 pin 14 (A), pin 16 (B) (right-rear) */
	{
		TIM4,
		GPIOB,
		GPIO_PIN_6,
		GPIOB,
		GPIO_PIN_7,
		GPIO_AF2_TIM4,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM4EN),
		0U
	},
	/* M4 RF — TIM5, PA0/PA1, AF2 - CN11 pin 28 (A), pin 30 (B) [morpho header]
	 * PA1 is driven by the Ethernet PHY 50 MHz RMII clock via solder bridge SB57
	 * (ON by default on Nucleo-H723ZG). ic_filter=15 rejects edges shorter than
	 * ~2.67 µs (8 samples at fDTS/32 = 3 MHz), blocking the 50 MHz phantom signal
	 * while passing real encoder edges (half-period >>78 µs at max motor speed).
	 * Permanent fix: desolder SB57. */
	{
		TIM5,
		GPIOA,
		GPIO_PIN_0,
		GPIOA,
		GPIO_PIN_1,
		GPIO_AF2_TIM5,
		ENCODER_RCC_ENCODE(ENCODER_RCC_BUS_APB1L, RCC_APB1LENR_TIM5EN),
		15U
	}
};

#endif /* ENCODER_CONFIG_H */
