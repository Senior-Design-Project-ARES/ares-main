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


/**
 * @brief System clock configuration for NUCLEO-H723ZG
 * 
 * System Clock: 520 MHz
 * HSE: 8 MHz (bypass mode)
 * PLL: HSE -> PLL -> System Clock
 * 
 * THIS IS NOT USED, BUT IS HERE FOR REFERENCE
 */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {}; // used to be {0} -- redefined to see if this fixes errors
    RCC_OscInitTypeDef RCC_OscInitStruct = {}; // used to be {0} -- redefined to see if this fixes errors
    HAL_StatusTypeDef ret = HAL_OK;

    // Configure voltage scaling
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    // Configure HSE and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 260;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if(ret != HAL_OK) {
        while(1) {} // Error handler
    }

    // Configure system clock and bus dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
                                   RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | 
                                   RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);
    
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
    if(ret != HAL_OK) {
        while(1) {} // Error handler
    }
}


/**
 * @brief Configure GPIO for LEDs
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure Yellow LED (PE1)
    GPIO_InitStruct.Pin = LED_YELLOW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_YELLOW_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET); // LED off
    
    // Configure Red LED (PB14)
    GPIO_InitStruct.Pin = LED_RED_PIN;
    HAL_GPIO_Init(LED_RED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET); // LED off
}


/**
 * @brief Setup function - runs once at startup (like Arduino setup())
 */
void setup()
{
    // HAL initialization
    HAL_Init();
    
    // Configure system clock
    // SystemClock_Config();
    
    // HAL_Delay uses SysTick which is configured by HAL_Init()
    // SystemCoreClock is updated by SystemClock_Config(), so delays will be accurate
    
    // Enable I-Cache and D-Cache for better performance
    // SCB_EnableICache();
    // SCB_EnableDCache();

    // Add your initialization code here
    // GPIO, UART, TIM, ETH configuration goes here
    
    MX_GPIO_Init(); // Configure GPIO for LEDs
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
    // Add your main application code here
    // This runs continuously after setup()

    // Turn off Yellow LED, turn on Red LED
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET); // Yello LED off
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);         // Red LED on
    HAL_Delay(1000);  // Wait 1 seconds

    // Turn on Yellow LED, turn off Red LED
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);   // Yello LED on
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);       // Red LED off
    HAL_Delay(1000);  // Wait 1 seconds
    
    // Cycle repeats (total 5 seconds per cycle)
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

    // HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);
    // while(1){}

    // HAL_Init();
    // MX_GPIO_Init();

    // while (1)
    // {
    //     HAL_GPIO_TogglePin(LED_YELLOW_PORT, LED_YELLOW_PIN);

    //     // for(volatile int i=0;i<1000000;i++);
    //     HAL_Delay(1000);
    // }
    
    return 0;
}

// Minimal C++ initialization stub for embedded systems
extern "C" {
    void _init(void) {
        // No C++ global constructors needed for embedded
    }
}
