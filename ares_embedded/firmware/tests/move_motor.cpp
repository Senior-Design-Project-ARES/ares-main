/**
 * @file test_motor_driver.cpp
 * @brief Minimal MC33926 motor driver smoke test: all wheels forward at 10% duty.
 * @author Vishnu Duriseti
 *
 * Uses Drivers/motor (motor_driver.h + motor_config.h). Serial line on USART3
 * @ 115200 prints a one-line banner; wheels keep running until power-off or reset.
 */

#include <cstdint>

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "motor_config.h"
#include "motor_driver.h"

#include "uart_driver.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

namespace {

constexpr uint8_t kDutyPercent = 70; // 10% duty cycle

} // namespace

int main(void)
{
    HAL_Init();
    uart_driver_init_default();

    static TIM_HandleTypeDef htim_pwm = {};
    motor_timer_init(&htim_pwm, MOTOR_PWM_TIMER, MOTOR_PWM_HZ, MOTOR_TIMER_CLK_HZ);

    static motor_driver_config_t configs[MOTOR_COUNT] = MOTOR_DRIVER_CONFIGS(&htim_pwm);
    static motor_driver_t        motors[MOTOR_COUNT]  = {};

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_driver_init(&configs[i], &motors[i]);
    }

    println("motor test: MOTOR_FORWARD %u%% on all channels (MC33926 driver)", static_cast<unsigned>(kDutyPercent));

    while (true) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            motor_drive(&motors[i], MOTOR_FORWARD, kDutyPercent);
        }
        HAL_Delay(50);
    }
}

extern "C" {
void _init(void) {}
}
