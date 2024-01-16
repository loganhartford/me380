#include "stm32f4xx.h"  // Include the STM32F4xx HAL library
#include "stm32f4xx_hal.h"


// Function prototypes
void SystemClock_Config(void);
void TIM3_Init(void);
void GPIO_Init(void);

// TIM handle declaration
TIM_HandleTypeDef htim3;

int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // // Configure the system clock
    // SystemClock_Config();

    // Initialize GPIO for PWM
    GPIO_Init();

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


    // PWM TEST
    // Initialize TIM3 for PWM
    TIM3_Init();

    // Start PWM signal on channel 1 of TIM3
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);



    // Main program logic
    while (1) {
        // Your code here
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

        // Increase duty cycle from 0 to Period
        for(uint32_t duty = 0; duty < htim3.Init.Period; duty++) {
            // duty++;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // Update the duty cycle
            HAL_Delay(1); // Delay for a short period
        }

        // Decrease duty cycle from Period to 0
        for(uint32_t duty = htim3.Init.Period; duty > 0; duty--) {
            // duty--;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // Update the duty cycle
            HAL_Delay(1); // Delay for a short period
        }
    }
}

void GPIO_Init(void) {
    // Enable GPIOB clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO pin : PB4 for PWM output 
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Set as alternate function push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;  // Set alternate function to TIM3
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TIM3_Init(void) {
    // Enable TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;  // Prescaler value
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;  // Adjust this value to change the PWM frequency
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    // PWM configuration for TIM3 Channel 1
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  // Adjust this value to change the duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
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