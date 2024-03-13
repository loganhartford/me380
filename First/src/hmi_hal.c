#include "hmi_hal.h"
// #include "main.h"

TIM_HandleTypeDef htim5;

void HMI_Init(void);
static void TIM5_Init(void);
void TIM5_IRQHandler(void);
// void changeLEDState(buttonLED butLED, const char *ledMode);
void buttonDebug(void);

//   More function prototypes

buttonLED greenLED =
    {
        .name = "greenLED",
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED redLED =
    {
        .name = "redLED",
        .port = GPIOC,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

// For TIM5 interrupt
buttonLED activeLED =
    {
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED homeButton =
    {
        .name = "homeButton",
        .port = GPIOC,
        .pin = GPIO_PIN_11,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED runTestButton =
    {
        .name = "runTestButton",
        .port = GPIOC,
        .pin = GPIO_PIN_10,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED autoManButton =
    {
        .name = "autoManButton",
        .port = GPIOD,
        .pin = GPIO_PIN_2,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

joystick controlJoystick = // UPDATE W/ ABOVE
    {
        .name = "controlJoystick",
        .port = GPIOA,
        .Pin_grip = GPIO_PIN_4,
        .Pin_x = ADC_CHANNEL_0,
        .Pin_y = ADC_CHANNEL_1,
};
// Create the other objects

/**
 * @brief Initializes the pins and state of the buttons/LEDs
 *
 * @param butLED Button/LED object
 */
void buttonLED_Init(buttonLED *butLED)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Pin configuration
    GPIO_InitStruct.Pin = butLED->pin;
    GPIO_InitStruct.Mode = butLED->mode;
    GPIO_InitStruct.Pull = butLED->pull;
    GPIO_InitStruct.Speed = butLED->speed;
    HAL_GPIO_Init(butLED->port, &GPIO_InitStruct);

    butLED->pin_state = HAL_GPIO_ReadPin(butLED->port, butLED->pin);
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
    TIM5_Init();

    buttonLED_Init(&greenLED);
    buttonLED_Init(&redLED);
    buttonLED_Init(&homeButton);
    buttonLED_Init(&runTestButton);
    buttonLED_Init(&autoManButton);

    // Joystick_Init(&controlJoystick);

    // Pot

    // Screen
}

static void TIM5_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFF; // 0.5s / inv(1MHz)
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim5);

    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
            HAL_GPIO_TogglePin(activeLED.port, activeLED.pin);
        }
    }
}

void flashLED(buttonLED butLED, double speed)
{
    double timerPeriod = 1000000 * speed; // 1MHz clock * Speed
    activeLED.port = butLED.port;
    activeLED.pin = butLED.pin;
    __HAL_TIM_SetCounter(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, timerPeriod);
    HAL_TIM_Base_Start_IT(&htim5);
}

void solidLED(buttonLED butLED)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_GPIO_WritePin(butLED.port, butLED.pin, GPIO_PIN_SET);
}

void stopLED(buttonLED butLED)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_GPIO_WritePin(butLED.port, butLED.pin, GPIO_PIN_RESET);
}

/**
 * @brief Changes state of LEDs
 *
 * @param butLED Button/LED object
 * @param ledMode Slow, Fast, or Solid
 */
void changeLEDState(buttonLED butLED, const char *ledMode)
{
    if (strcmp(butLED.name, "greenLED") == 0)
    {
        if (strcmp(ledMode, "Slow") == 0)
        {
            stopLED(redLED);
            flashLED(greenLED, 0.5);
        }
        else if (strcmp(ledMode, "Fast") == 0)
        {
            stopLED(redLED);
            flashLED(greenLED, 0.1);
        }
        else
        {
            stopLED(redLED);
            solidLED(greenLED);
        }
    }
    else
    {
        if (strcmp(ledMode, "Slow") == 0)
        {
            stopLED(greenLED);
            flashLED(redLED, 0.5);
        }
        else if (strcmp(ledMode, "Fast") == 0)
        {
            stopLED(greenLED);
            flashLED(redLED, 0.1);
        }
        else
        {
            stopLED(greenLED);
            solidLED(redLED);
        }
    }
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
 * @brief Print pin state of 3 buttons to console for debugging
 *
 */
void buttonDebug(void)
{
    while (1)
    {
        readDigitalPinState(homeButton);
        readDigitalPinState(runTestButton);
        readDigitalPinState(autoManButton);
        printf("\n\r");
        HAL_Delay(2000);
    }
}