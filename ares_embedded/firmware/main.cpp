/**
 * @file main.cpp
 * @brief Minimal firmware entry point for STM32H723ZG
 * @author Vishnu Duriseti
 * Arduino-like workflow: setup() runs once, loop() runs forever
 */

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

extern "C" {
    void SystemClock_Config(void);
    void SystemInit(void);
    void ExitRun0Mode(void);
}

// LED GPIO definitions for NUCLEO-H723ZG
// LD2 (Yellow): PE1
// LD3 (Red): PB14
#define LED_YELLOW_PORT    GPIOE
#define LED_YELLOW_PIN     GPIO_PIN_1
#define LED_RED_PORT       GPIOB
#define LED_RED_PIN        GPIO_PIN_14

// Global timer handle for red LED PWM
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
 * @brief Setup function - runs once at startup (like Arduino setup())
 */
void setup()
{
    // HAL initialization
    HAL_Init();
    
    // GPIO, UART, TIM, ETH configuration goes here
    yellow_led_init();
    red_led_init(); 
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
    setup();
    
    while(true) {
        loop();
    }

    return 0;
}

// Minimal C++ initialization stub for embedded systems
extern "C" {
    void _init(void) {
        // No C++ global constructors needed for embedded
    }
}
