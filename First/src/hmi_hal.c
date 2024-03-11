#include "hmi_hal.h"
// #include "main.h"

buttonLED greenLED =
    {
        .name = "greenLED",
        .port = GPIOA,
        .pin = GPIO_PIN_0,
};

// Create the other objects

/**
 * @brief Initializes the pins and state of the LED or input button
 *
 * @param butLED Button or LED object
 */
void buttonLED_Init(buttonLED *butLED)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Pin configuration
    GPIO_InitStruct.Pin = butLED->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(butLED->port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(butLED->port, butLED->pin, GPIO_PIN_SET);
}