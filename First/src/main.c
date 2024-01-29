#include "stm32f4xx.h" // Include the STM32F4xx HAL library
#include "stm32f4xx_hal.h"

// Function prototypes
void SystemClock_Config(void);
void TIM3_Init(void);
void GPIO_Init(void);

#define IN1_PIN GPIO_PIN_0
#define IN2_PIN GPIO_PIN_1
#define IN3_PIN GPIO_PIN_2
#define IN4_PIN GPIO_PIN_3

// Function to initialize GPIO pins
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure IN1, IN2, IN3, and IN4 pins as outputs
    GPIO_InitStruct.Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
// TIM handle declaration
TIM_HandleTypeDef htim3;

void DriveStepperMotor(void)
{
    // Step 1
    HAL_GPIO_WritePin(GPIOA, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 2
    HAL_GPIO_WritePin(GPIOA, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 3
    HAL_GPIO_WritePin(GPIOA, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN4_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 4
    HAL_GPIO_WritePin(GPIOA, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

int main(void)
{
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    // SystemClock_Config();

    // Initialize GPIO for stepper motor control
    GPIO_Init();

    // Main program logic
<<<<<<< HEAD
    while (1)
    {
        // Drive the stepper motor in a continuous loop
        DriveStepperMotor();
        HAL_Delay(100); // Adjust delay based on your motor's speed requirements
=======
    while (1) {
        // Your code here
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

        // Increase duty cycle from 0 to Period
        for(uint32_t duty = 0; duty < htim3.Init.Period; duty++) {
            // duty++;
            printf(duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // Update the duty cycle
            HAL_Delay(1); // Delay for a short period
        }

        // Decrease duty cycle from Period to 0
        for(uint32_t duty = htim3.Init.Period; duty > 0; duty--) {
            // duty--;
            printf(duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // Update the duty cycle
            HAL_Delay(1); // Delay for a short period
        }
>>>>>>> 1cf70ae1aa04af9d8af061e4e921cd396bf9456a
    }
}

void TIM3_Init(void)
{
    // Enable TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0; // Prescaler value
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999; // Adjust this value to change the PWM frequency
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    // PWM configuration for TIM3 Channel 1
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500; // Adjust this value to change the duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}
// When error is encountered. Error_Handler is used to halt the program - stops microcontroller
// from doing any other instuctions
void Error_Handler(void)
{
    // Error handling code
    while (1)
    {
        // Infinite loop or other error handling mechanism
    }
}
// this is an ISR, used by the HAL to track milliseconds
void SysTick_Handler(void)
{
    HAL_IncTick();
}