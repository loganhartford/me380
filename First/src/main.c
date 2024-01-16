#include "stm32f4xx.h"  // Include the STM32F4xx HAL library
#include "stm32f4xx_hal.h"


// Function prototypes
void SystemClock_Config(void);

int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // // Configure the system clock
    // SystemClock_Config();

    // Enable the clock for the GPIO port (in this case, GPIOB)
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure Pin PB3 as a general-purpose output
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Turn on Pin PB3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);


    // Main program logic
    while (1) {
        // Your code here
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    }
}

// // System Clock Configuration
// void SystemClock_Config(void) {
//     RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//     RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//     __HAL_RCC_PWR_CLK_ENABLE();
//     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//     RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//     RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//     RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//     RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//     RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//     RCC_OscInitStruct.PLL.PLLM = 16;
//     RCC_OscInitStruct.PLL.PLLN = 336;
//     RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//     RCC_OscInitStruct.PLL.PLLQ = 7;
//     if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//         Error_Handler();
//     }

//     RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//     RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//     RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//     RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//     RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//     if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
//         Error_Handler();
//     }
// }

void Error_Handler(void) {
    // Error handling code
    while (1) {
        // Infinite loop or other error handling mechanism
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}