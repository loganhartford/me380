#include "hmi_hal.h"
#include "motor_hal.h"
#include "controls.h"

TIM_HandleTypeDef htim5;
ADC_HandleTypeDef hadc1;

void HMI_Init(void);
static void TIM5_Init(void);
void TIM5_IRQHandler(void);
// void changeLEDState(buttonLED butLED, const char *ledMode);
void buttonDebug(void);
void Pot_Init(Pot *pot);
void ADC_Init(void);
void ADC_Select_Channel(uint32_t channel);

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

Pot xPot =
    {
        .name = "xPot",
        .port = GPIOA,
        .pin = GPIO_PIN_4,
        .mode = GPIO_MODE_ANALOG,
        .pull = GPIO_NOPULL,
        .channel = ADC_CHANNEL_4};

Pot yPot =
    {
        .name = "yPot",
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_ANALOG,
        .pull = GPIO_NOPULL,
        .channel = ADC_CHANNEL_1};

Pot zPot =
    {
        .name = "zPot",
        .port = GPIOB,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_ANALOG,
        .pull = GPIO_NOPULL,
        .channel = ADC_CHANNEL_8};

buttonLED gripButton = {
    .name = "gripButton",
    .port = GPIOA,
    .pin = GPIO_PIN_0,
    .mode = GPIO_MODE_INPUT,
    .pull = GPIO_NOPULL,
    .speed = GPIO_SPEED_FREQ_LOW,
};

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

void Pot_Init(Pot *pot)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pot->pin;
    GPIO_InitStruct.Mode = pot->mode;
    GPIO_InitStruct.Pull = pot->pull;
    HAL_GPIO_Init(pot->port, &GPIO_InitStruct);

    pot->alpha = 0.1;
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
    buttonLED_Init(&gripButton);

    // Pots
    Pot_Init(&xPot);
    Pot_Init(&yPot);
    Pot_Init(&zPot);
    ADC_Init();

    zPot.max = 860;
    zPot.min = 5;
    zPot.slope = ((motorz.thetaMax - Z_SAFETY_MARGIN) - (motorz.thetaMin + Z_SAFETY_MARGIN)) / (zPot.max - zPot.min);
    zPot.b = (motorz.thetaMax - Z_SAFETY_MARGIN) - (zPot.slope * zPot.max);

    xPot.value = Read_Pot(&xPot);
    yPot.value = Read_Pot(&yPot);
    zPot.value = Read_Pot(&zPot);
}

void ADC_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        ErrorHandler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        ErrorHandler();
    }
}

void ADC_Select_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        ErrorHandler();
    }
}

uint32_t Read_Pot(Pot *pot)
{
    ADC_Select_Channel(pot->channel); // Select the channel before reading

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        return HAL_ADC_GetValue(&hadc1);
    }
    return 0; // Return 0 if failed
}

void readAndFilter(Pot *pot)
{
    pot->value = Read_Pot(pot);
    if (!pot->filtered)
    {
        pot->filtered = pot->value;
    }
    else
    {
        pot->filtered = pot->filtered + (((pot->value - pot->filtered) * pot->alpha));
    }
}

void Manual_Mode(void)
{
    gripButton.latched = 0;
    double lastZPos = 0;
    while (1)
    {
        // Read user inputs
        // xPot.filtered = xPot.filtered + ((Read_Pot(&xPot) - xPot.filtered) * xPot.alpha);
        readAndFilter(&xPot);
        readAndFilter(&yPot);
        readAndFilter(&zPot);
        gripButton.pin_state = HAL_GPIO_ReadPin(gripButton.port, gripButton.pin);

        // Actuate the gripper
        if (!gripButton.pin_state && !gripButton.latched)
        {
            gripButton.latched = 1;
            if (gripper.isOpen)
            {
                printf("open\r\n");
                gripperClose(&gripper);
            }
            else
            {
                gripperOpen(&gripper);
                printf("close\r\n");
            }
        }
        else if (gripButton.pin_state)
        {
            gripButton.latched = 0;
        }

        // Move the z axis
        double zPos = zPot.slope * zPot.filtered + zPot.b;
        if (zPos > (motorz.thetaMax - Z_SAFETY_MARGIN))
        {
            zPos = (motorz.thetaMax - Z_SAFETY_MARGIN) - 1;
        }
        else if (zPos < (motorz.thetaMin + Z_SAFETY_MARGIN))
        {
            zPos = (motorz.thetaMin + Z_SAFETY_MARGIN) + 1;
        }

        if ((fabs(zPos - state.currentZ) > 1.0) && (fabs(zPos - lastZPos) > 1.0))
        {
            PrintCaresianCoords(zPos, state.currentZ);
            MoveToZ(zPos, 10.0);
        }
        lastZPos = zPos;
        // PrintCaresianCoords(zPot.slope, );

        // PrintCaresianCoords(xPot.value, yPot.value);
        // For simplicity, just take th higher speed as the overall speed
        double xSpeed = (fabs(xPot.filtered - 2048.0) / 2048.0) * 15.0 + 5.0;
        double ySpeed = (fabs(yPot.filtered - 2048.0) / 2048.0) * 15.0 + 5.0;
        // PrintCaresianCoords(xSpeed, ySpeed);
        double speed;
        if (xSpeed > ySpeed)
        {
            speed = xSpeed;
        }
        else
        {
            speed = ySpeed;
        }

        // For now just make x an y coords binary, can add scaling once everything else works
        double x, y = 0;
        if ((xPot.value - 2048.0) > 1000.0)
        {
            x = 1;
        }
        else if ((xPot.value - 2048.0) < -1000.0)
        {
            x = -1;
        }
        else
        {
            x = 0;
        }
        if ((yPot.value - 2048.0) > 1000.0)
        {
            y = 1;
        }
        else if ((yPot.value - 2048.0) < -1000.0)
        {
            y = -1;
        }
        else
        {
            y = 0;
        }

        // Send move command

        // PrintCaresianCoords(x, y);

        /*

        - for position, linear interpolate on each axis from like 1-5 and then add a short delay to the loop to allow for the step discrepancy to matter, might need to play with the delay so that it's long engough for the motor's to
        - right now the number of requested steps is returned and that is how the position is updated, but if we make another step request before the sequence is completed, it will call again and reset the timmer/motor and the state machine will think it has moved when it has not
        - would need change to be updating the state machine in the step loop so that only the steps that actually get complete get added
        - Then it shouldn't matter if we call it again
        - Same thing with z axis
        - Implement this first and make sure the demo still works
        -
        */
        // Return to automatic mode
        if (HAL_GPIO_ReadPin(autoManButton.port, autoManButton.pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(500); // So button isn't "double-pressed"
            return;
        }
        HAL_Delay(20);
    }
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