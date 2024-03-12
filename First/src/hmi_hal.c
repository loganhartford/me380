#include "hmi_hal.h"
// #include "main.h"

ADC_HandleTypeDef hadc1;

void HMI_Init(void);
void readDigitalPinState(buttonLED butLED);
//  More function prototypes

buttonLED greenLED =
    {
        .name = "greenLED",
        .port = GPIOC,
        .pin = GPIO_PIN_0,
};

buttonLED redLED =
    {
        .name = "redLED",
        .port = GPIOC,
        .pin = GPIO_PIN_1,
};

buttonLED homeButton =
    {
        .name = "homeButton",
        .port = GPIOC,
        .pin = GPIO_PIN_11,
};

buttonLED runTestButton =
    {
        .name = "runTestButton",
        .port = GPIOC,
        .pin = GPIO_PIN_14,
};

buttonLED autoManButton =
    {
        .name = "autoManButton",
        .port = GPIOD,
        .pin = GPIO_PIN_2,
};

joystick controlJoystick =
    {
        .name = "controlJoystick",
        .port = GPIOA,
        .Pin_grip = GPIO_PIN_4,
        .Pin_x = ADC_CHANNEL_0,
        .Pin_y = ADC_CHANNEL_1,
};
// Create the other objects

/**
 * @brief Initializes the pins and state of the LEDs
 *
 * @param butLED Button/LED object
 */
void LED_Init(buttonLED *butLED)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Pin configuration
    GPIO_InitStruct.Pin = butLED->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(butLED->port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(butLED->port, butLED->pin, 0);
}

/**
 * @brief Initializes the pins and state of the HMI buttons
 *
 * @param butLED Button/LED object
 */
void Button_Init(buttonLED *butLED)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Pin configuration
    GPIO_InitStruct.Pin = butLED->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(butLED->port, &GPIO_InitStruct);
}

/**
 * @brief Initializes the pins and state of the HMI joystick
 *
 * @param stick Joystick object
 */
void Joystick_Init(joystick *stick)
{
    // DIGITAL Initialization
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Knob Button
    GPIO_InitStruct.Pin = stick->Pin_grip;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(stick->port, &GPIO_InitStruct);

    // ANALOG Initializaton - ADD
}

/**
 * @brief Initializes all HMI components, to be used in main
 *
 */
void HMI_Init(void)
{
    LED_Init(&greenLED);
    LED_Init(&redLED);

    Button_Init(&homeButton);
    Button_Init(&runTestButton);
    Button_Init(&autoManButton);

    Joystick_Init(&controlJoystick);

    // Pot

    // Screen
}

void updateStateLED(buttonLED butLED)
{
}

void readDigitalPinState(buttonLED butLED)
{
    GPIO_PinState button_state = HAL_GPIO_ReadPin(butLED.port, butLED.pin);
    const char *button_name = butLED.name;

    if (button_state == GPIO_PIN_SET)
    {
        printf("%s INACTIVE\n\r", button_name);
    }
    else if (button_state == GPIO_PIN_RESET)
    {
        printf("%s ACTIVE\n\r", button_name);
    }
}

/**
 * @brief Changes state LEDs to reflect robot state
 *
 * @param butLED button/LED object
 * @param int State #
 */
