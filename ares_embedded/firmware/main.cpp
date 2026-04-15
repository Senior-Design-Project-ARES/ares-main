/**
 * @file main.cpp
 * @brief Minimal firmware entry point for STM32H723ZG
 * @author Vishnu Duriseti
 * Arduino-like workflow: setup() runs once, loop() runs forever
 */

#include <cstdint>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio_ex.h"
#include "ares_control.hpp" // custom control library
#include "encoder_driver.h" // encoder driver
#include "uart_driver.h"
#include "ethernet_driver.h" // ethernet driver

extern "C" {
    void SystemClock_Config(void);
    void SystemInit(void);
    void ExitRun0Mode(void);
}

#define LED_YELLOW_PORT    GPIOE
#define LED_YELLOW_PIN     GPIO_PIN_1
#define LED_RED_PORT       GPIOB
#define LED_RED_PIN        GPIO_PIN_14

// DRV8256P truth: IN1=PWM,IN2=0 → forward; IN1=0,IN2=PWM → reverse; both 0 → brake low; both 1 → brake high

/** Pin + AF for one motor driver input (IN1 or IN2) */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t     pin;
    uint8_t      alternate;  /* e.g. GPIO_AF1_TIM1 */
} motor_pin_t;

/**
 * Config for one motor. IN1 and IN2 may be on different timers
 * (e.g. IN1 on TIM1, IN2 on TIM2). Timer handles are shared and
 * initialized separately via motor_timer_init<PWM_HZ>().
 */
typedef struct {
    motor_pin_t        in1;
    motor_pin_t        in2;
    TIM_HandleTypeDef *htim_in1;    /* pointer to shared timer handle for IN1 */
    TIM_HandleTypeDef *htim_in2;    /* pointer to shared timer handle for IN2 */
    uint32_t           channel_in1;
    uint32_t           channel_in2;
} motor_driver_config_t;

/** Runtime context for one motor, filled by motor_driver_init */
typedef struct {
    TIM_HandleTypeDef *htim_in1;
    TIM_HandleTypeDef *htim_in2;
    uint32_t           period;   /* ARR (same for both timers when same frequency) */
    uint32_t           ch_in1;
    uint32_t           ch_in2;
} motor_driver_t;

// Global timer handle for red LED
TIM_HandleTypeDef htim12_red_led = {};


/**
 * @brief Configure GPIO for LEDs
 */
void yellow_led_init(void)
{
    // GPIO Initialization Structure
    GPIO_InitTypeDef LD2 = {}; // LED pin struct for LD2 (Yellow LED)
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // Configure Yellow LED (PE1) -- CANNOT do PWM on this pin (no physical TIM peripheral)
    LD2.Pin = LED_YELLOW_PIN;
    LD2.Mode = GPIO_MODE_OUTPUT_PP;
    LD2.Pull = GPIO_NOPULL;
    LD2.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_YELLOW_PORT, &LD2);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET); // LED off
}

/**
 * @brief TIM12 PWM setup for LD3 (PB14)
 *
 * Hardware idea:
 *   Timer counts: 0 → Period → repeat
 *   Output HIGH while counter < compare value
 *   Output LOW  otherwise
 *
 * So:
 *   Period  = resolution (max brightness)
 *   Compare = duty cycle (brightness)
 *
 * This behaves like Arduino analogWrite(0–255)
 */
void red_led_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();   // power to GPIO port B
    __HAL_RCC_TIM12_CLK_ENABLE();   // power to TIM12 hardware block

    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_14;
    g.Mode      = GPIO_MODE_AF_PP; // Alternate function mode means - "timer peripheral controls this pin, not the CPU"
    g.Pull      = GPIO_NOPULL;  
    g.Speed     = GPIO_SPEED_FREQ_LOW; // Speed only affects edge sharpness, not frequency  
    g.Alternate = GPIO_AF2_TIM12; // AF2 maps PB14 → TIM12_CH1 (from datasheet table)
    HAL_GPIO_Init(GPIOB, &g);


    // Timer configuration
    htim12_red_led.Instance = TIM12;   // choose physical timer block TIM12

    /*
      PRESCALER
      ----------
      Divides timer input clock.

      timer_tick = timer_clock / (Prescaler + 1)

      Example:
        timer_clock = 64 MHz
        prescaler = 63

        → 64 MHz / 64 = 1 MHz

      So counter increments 1,000,000 times/sec.
    */
    htim12_red_led.Init.Prescaler = 64 - 1;

    /*
      PERIOD (ARR)
      ------------
      Maximum counter value before reset.

      Counter counts:
        0 → Period → 0 → Period → ...

      Also defines PWM resolution.

      Period = 255
        → 256 brightness levels
        → identical to Arduino analogWrite 0–255
    */
    htim12_red_led.Init.Period = 255;

    /*
      COUNTER MODE
      ------------
      UP:
        0 → max → reset
        normal PWM

      DOWN:
        max → 0 → reset

      CENTER_ALIGNED:
        0 → max → 0 (symmetric, motor control)

      For LEDs, always use UP.
    */
    htim12_red_led.Init.CounterMode = TIM_COUNTERMODE_UP;

    /*
      Initialize timer hardware registers
      (writes prescaler/period into silicon)
    */
    HAL_TIM_PWM_Init(&htim12_red_led);



    // PWM channel config
    TIM_OC_InitTypeDef s = {};

    /*
      PWM1 mode:
        output HIGH while counter < compare
        output LOW otherwise
      (standard duty-cycle behavior)
    */
    s.OCMode = TIM_OCMODE_PWM1;
    s.Pulse = 0;   // start at 0% brightness
    s.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim12_red_led, &s, TIM_CHANNEL_1);

    // Start timer + PWM signal generation
    HAL_TIM_PWM_Start(&htim12_red_led, TIM_CHANNEL_1);
}

/**
 * @brief TIM1 PWM for DRV8256P motor driver (D5=PE9/IN1, D6=PE11/IN2), 1 kHz
 */

/** DRV8256P modes per truth table: IN1, IN2 → outcome */
typedef enum {
    MOTOR_BRAKE_LOW  = 0,  /* IN1=0, IN2=0 */
    MOTOR_FORWARD    = 1,  /* IN1=PWM, IN2=0 */
    MOTOR_REVERSE    = 2,  /* IN1=0, IN2=PWM */
    MOTOR_BRAKE_HIGH = 3,  /* IN1=1, IN2=1 */
} motor_mode_t;

static void motor_gpio_clk_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}

static void motor_tim_clk_enable(TIM_TypeDef *tim)
{
    if (tim == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (tim == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

/**
 * @brief Init a timer for motor PWM. Call once per timer, before motor_driver_init.
 *
 * Usage: motor_timer_init<20000u>(&htim, TIM1, 64000000u);
 *
 * PWM_HZ must be 10-40 kHz or the build fails (compile-time check).
 * PSC = 0 for maximum duty-cycle resolution.
 */
template <uint32_t PWM_HZ>
void motor_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t timer_clk_hz)
{
    static_assert(PWM_HZ >= 10000u && PWM_HZ <= 40000u,
                  "Motor PWM frequency must be between 10 kHz and 40 kHz to avoid overflow");

    motor_tim_clk_enable(instance);

    htim->Instance               = instance;
    htim->Init.Prescaler         = 0u;
    htim->Init.Period            = (timer_clk_hz / PWM_HZ) - 1u;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0u;
    HAL_TIM_PWM_Init(htim);
}

/**
 * @brief Init one motor (GPIO pins + PWM channels). Timers must already be
 *        initialized via motor_timer_init<PWM_HZ>(). IN1 and IN2 may be on
 *        different timers.
 */
void motor_driver_init(const motor_driver_config_t *config, motor_driver_t *ctx)
{
    if (!config || !ctx) return;

    /* Enable GPIO clocks and configure AF pins */
    motor_gpio_clk_enable(config->in1.port);
    motor_gpio_clk_enable(config->in2.port);

    GPIO_InitTypeDef g = {};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin       = config->in1.pin;
    g.Alternate = config->in1.alternate;
    HAL_GPIO_Init(config->in1.port, &g);

    g.Pin       = config->in2.pin;
    g.Alternate = config->in2.alternate;
    HAL_GPIO_Init(config->in2.port, &g);

    /* Configure and start PWM channels (timer base already running) */
    TIM_OC_InitTypeDef oc = {};
    oc.OCMode   = TIM_OCMODE_PWM1;
    oc.Pulse    = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;

    HAL_TIM_PWM_ConfigChannel(config->htim_in1, &oc, config->channel_in1);
    HAL_TIM_PWM_Start(config->htim_in1, config->channel_in1);

    HAL_TIM_PWM_ConfigChannel(config->htim_in2, &oc, config->channel_in2);
    HAL_TIM_PWM_Start(config->htim_in2, config->channel_in2);

    /* Fill runtime context */
    ctx->htim_in1 = config->htim_in1;
    ctx->htim_in2 = config->htim_in2;
    ctx->period   = config->htim_in1->Init.Period;  /* same freq → same period */
    ctx->ch_in1   = config->channel_in1;
    ctx->ch_in2   = config->channel_in2;
}

/**
 * @brief Set motor direction and duty cycle (0–100%). Brake modes ignore duty.
 */
void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent)
{
    if (!ctx) return;
    if (duty_percent > 100u) duty_percent = 100u;

    uint32_t pulse = (uint32_t)duty_percent * (ctx->period + 1u) / 100u;
    if (pulse > ctx->period) pulse = ctx->period;

    switch (mode) {
        case MOTOR_BRAKE_LOW:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, 0);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, 0);
            break;
        case MOTOR_FORWARD:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, pulse);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, 0);
            break;
        case MOTOR_REVERSE:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, 0);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, pulse);
            break;
        case MOTOR_BRAKE_HIGH:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, ctx->period);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, ctx->period);
            break;
    }
}

/**
 * @brief Loop function - runs forever (like Arduino loop())
 * 
 * Blinking pattern:
 * - Yellow LED (LD2) on for 2.5 seconds
 * - Red LED (LD3) on for 2.5 seconds
 * - Cycle repeats every 5 seconds
 */
void loop()
{
    // This runs continuously after setup()

    // Turn off Yellow LED, fade Red LED in and out using PWM
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET); // Yellow LED off
    
    // Fade Red LED from 0 to 255 over 2.5 seconds
    const uint32_t fade_duration_ms = 2500;  // 2.5 seconds
    const uint32_t steps = 255;
    const uint32_t step_delay_ms = fade_duration_ms / steps;
    
    for (uint32_t i = 0; i <= steps; i++) {
        __HAL_TIM_SET_COMPARE(&htim12_red_led, TIM_CHANNEL_1, i);
        HAL_Delay(step_delay_ms);
    }
    
    // Fade Red LED from 255 to 0 over 2.5 seconds
    for (uint32_t i = steps; i > 0; i--) {
        __HAL_TIM_SET_COMPARE(&htim12_red_led, TIM_CHANNEL_1, i - 1);
        HAL_Delay(step_delay_ms);
    }
     __HAL_TIM_SET_COMPARE(&htim12_red_led, TIM_CHANNEL_1, 0);   // Ensure Red LED is fully off

    // Turn on Yellow LED, turn off Red LED
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);   // Yello LED on
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);       // Red LED off
    HAL_Delay(5000);  
    
    // Cycle repeats (total 10 seconds per cycle)
}

/**
 * @brief Main entry point
 */
int main(void)
{
    // HAL initialization
    HAL_Init();
    uart_driver_init_default(); /* USART3 PD8/9 @ 115200 (ST-Link VCP style) */

    yellow_led_init();
    red_led_init();

    /* Ethernet + LwIP bring-up (DHCP + TCP echo on port 7) */
    // ethernet_driver_config_t eth_cfg = {};
    // eth_cfg.use_dhcp = 1;
    // eth_cfg.tcp_port = 7;
    // ethernet_driver_init(&eth_cfg);

    /*
     * Motor timer setup: TIM1 (16-bit) and TIM2 (32-bit), both at 20 kHz.
     * Each motor uses one TIM1 channel (IN1) + one TIM2 channel (IN2).
     *
     * TIM1 channels (GPIOE, AF1):       TIM2 channels (GPIOA/B, AF1):
     *   D6  = PE9  = TIM1_CH1             D32 = PA0  = TIM2_CH1
     *   D5  = PE11 = TIM1_CH2             D33 = PA1  = TIM2_CH2
     *   D3  = PE13 = TIM1_CH3             D36 = PB10 = TIM2_CH3
     *   D9  = PE14 = TIM1_CH4             D35 = PB11 = TIM2_CH4
     */
    static TIM_HandleTypeDef htim1 = {};
    static TIM_HandleTypeDef htim2 = {};
    motor_timer_init<20000u>(&htim1, TIM1, 64000000u);
    motor_timer_init<20000u>(&htim2, TIM2, 64000000u);

    /* Motor 1: IN1 = D6  (PE9)  TIM1_CH1,  IN2 = D32 (PA0)  TIM2_CH1 */
    motor_driver_config_t m1_cfg = {
        .in1 = { GPIOE, GPIO_PIN_9,  GPIO_AF1_TIM1 },
        .in2 = { GPIOA, GPIO_PIN_0,  GPIO_AF1_TIM2 },
        .htim_in1    = &htim1,
        .htim_in2    = &htim2,
        .channel_in1 = TIM_CHANNEL_1,
        .channel_in2 = TIM_CHANNEL_1,
    };

    /* Motor 2: IN1 = D5  (PE11) TIM1_CH2,  IN2 = D33 (PA1)  TIM2_CH2 */
    motor_driver_config_t m2_cfg = {
        .in1 = { GPIOE, GPIO_PIN_11, GPIO_AF1_TIM1 },
        .in2 = { GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2 },
        .htim_in1    = &htim1,
        .htim_in2    = &htim2,
        .channel_in1 = TIM_CHANNEL_2,
        .channel_in2 = TIM_CHANNEL_2,
    };

    /* Motor 3: IN1 = D3  (PE13) TIM1_CH3,  IN2 = D36 (PB10) TIM2_CH3 */
    motor_driver_config_t m3_cfg = {
        .in1 = { GPIOE, GPIO_PIN_13, GPIO_AF1_TIM1 },
        .in2 = { GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2 },
        .htim_in1    = &htim1,
        .htim_in2    = &htim2,
        .channel_in1 = TIM_CHANNEL_3,
        .channel_in2 = TIM_CHANNEL_3,
    };

    /* Motor 4: IN1 = D9  (PE14) TIM1_CH4,  IN2 = D35 (PB11) TIM2_CH4 */
    motor_driver_config_t m4_cfg = {
        .in1 = { GPIOE, GPIO_PIN_14, GPIO_AF1_TIM1 },
        .in2 = { GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2 },
        .htim_in1    = &htim1,
        .htim_in2    = &htim2,
        .channel_in1 = TIM_CHANNEL_4,
        .channel_in2 = TIM_CHANNEL_4,
    };

    motor_driver_t motors[4] = {};
    motor_driver_init(&m1_cfg, &motors[0]);
    motor_driver_init(&m2_cfg, &motors[1]);
    motor_driver_init(&m3_cfg, &motors[2]);
    motor_driver_init(&m4_cfg, &motors[3]);
    uint32_t last_log_ms = HAL_GetTick();
    while (true) {
        // ethernet_driver_poll();
        uint32_t now_ms = HAL_GetTick();

        /* Simple debug print every second over UART */
        if ((now_ms - last_log_ms) >= 1000U) {
            last_log_ms = now_ms;
            println("Firmware alive, t=%lu ms", static_cast<unsigned long>(now_ms));
        }

        /* All 4 motors forward at 20% duty for 5 seconds */
        for (int i = 0; i < 4; i++)
            motor_drive(&motors[i], MOTOR_FORWARD, 20);
        HAL_Delay(5000);

        /* All 4 motors reverse at 20% duty for 5 seconds */
        for (int i = 0; i < 4; i++)
            motor_drive(&motors[i], MOTOR_REVERSE, 20);
        HAL_Delay(5000);
    }

    return 0;
}

// Minimal C++ initialization stub for embedded systems
extern "C" {
    void _init(void) {
        // No C++ global constructors needed for embedded
    }
}
