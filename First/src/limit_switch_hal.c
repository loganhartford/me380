#include "limit_switch_hal.h"
#include "motor_hal.h"

void EXTI1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

LimitSwitch limitSwitches =
    {
        .port = GPIOB,
        .theta1Pin = GPIO_PIN_1,
        .theta2Pin = GPIO_PIN_5,
        .thetazPin = GPIO_PIN_10,
};

void Limit_Switch_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // External interrupt pin configuration
    GPIO_InitStruct.Pin = limitSwitches.theta1Pin | limitSwitches.theta2Pin | limitSwitches.thetazPin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Trigger on rising edge
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(limitSwitches.port, &GPIO_InitStruct);

    // Enable and set EXTI line Interrupt to the given priority
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Define ISRs for the pins
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(limitSwitches.theta1Pin);

    motor1.limitTriggered = 1;

    // For testing only, delete later
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Toggle LED

}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(limitSwitches.theta2Pin);

    motor2.limitTriggered = 1;

    // For testing only, delete later
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Toggle LED
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(limitSwitches.thetazPin);

    motorz.limitTriggered = 1;

    // For testing only, delete later
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Toggle LED
}