#include "motor_hal.h"
#include "limit_switch_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void Motor_Init(Motor motor);
static void TIM3_Init(void);
static void TIM4_Init(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

// Motor Objects
Motor motor1 = {
    // X
    .stepPort = GPIOA,          // D2-PA10
    .stepPin = GPIO_PIN_10,     // D2-PA10
    .dirPort = GPIOB,           // D5-PB4
    .directionPin = GPIO_PIN_4, // D5-PB4
    .reduction = 1,
    .thetaMin = -160.0 / 180.0 * M_PI,
    .thetaMax = 160.0 / 180.0 * M_PI,
};

Motor motor2 = {
    // Y
    .stepPort = GPIOB,           // D3-PB3
    .stepPin = GPIO_PIN_3,       // D3-PB3
    .dirPort = GPIOB,            // D6-PB10
    .directionPin = GPIO_PIN_10, // D6-PB10
    .reduction = 2,
    .thetaMin = -100.0 / 180.0 * M_PI,
    .thetaMax = 100.0 / 180.0 * M_PI,
};

Motor motorz = {                // Z
    .stepPort = GPIOB,          // D4-PB5
    .stepPin = GPIO_PIN_5,      // D4-PB5
    .dirPort = GPIOA,           // D7-PA8
    .directionPin = GPIO_PIN_8, // D7-PA8
    .radsPerStep = 2 * M_PI / Z_STEPS_PER_REV,
    .reduction = 1};

/**
 * @brief Takes in a motor struct and initialized the associated pins.
 *
 * @param motor Motor struct
 */
void Motor_Init(Motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize Step Pin
    GPIO_InitStruct.Pin = motor.stepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(motor.stepPort, &GPIO_InitStruct);

    // Initialize Direction Pin
    GPIO_InitStruct.Pin = motor.directionPin;
    // Direction port may be the same as step port or different, hence a separate init
    HAL_GPIO_Init(motor.dirPort, &GPIO_InitStruct);
}

/**
 * @brief Initializes all three motors and starts the timer.
 *
 */
void Motors_Init(void)
{
    Motor_Init(motor1);
    Motor_Init(motor2);
    Motor_Init(motorz);
    TIM3_Init();
    TIM4_Init();
}

void MoveTheta1(double angle, double speedRPM)
{
    motor1.stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV);

    float timePerStep = 60.0 / (speedRPM * STEPS_PER_REV);              // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds

    __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
    HAL_TIM_Base_Start_IT(&htim3);
}

static void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            HAL_GPIO_TogglePin(motor1.stepPort, motor1.stepPin);
            motor1.stepsToComplete--;
            if (!motor1.stepsToComplete)
            {
                HAL_TIM_Base_Stop_IT(&htim3);
            }
        }
    }
}

void MoveTheta2(double angle, double speedRPM)
{
    motor2.stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV);

    float timePerStep = 60.0 / (speedRPM * STEPS_PER_REV);              // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds

    __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
    HAL_TIM_Base_Start_IT(&htim4);
}

static void TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
            HAL_GPIO_TogglePin(motor2.stepPort, motor2.stepPin);
            motor2.stepsToComplete--;
            if (!motor2.stepsToComplete)
            {
                HAL_TIM_Base_Stop_IT(&htim4);
            }
        }
    }
}

void HomeMotors(void)
{
    //    MoveByAngle(1, 1, 0.0, &realdelta1, &realdelta2, &realdeltaz);
}