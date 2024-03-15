#include "motor_hal.h"
#include "controls.h"
#include "limit_switch_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim2;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
static void TIM4_Init(void);
static void TIM2_Init(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);

// Motor Objects
Motor motor1 = {
    // X
    .name = "motor1",
    .stepPort = GPIOA,      // D2-PA10
    .stepPin = GPIO_PIN_10, // D2-PA10
    .dirPort = GPIOB,       // D5-PB4
    .dirPin = GPIO_PIN_4,   // D5-PB4
    .dir = CCW,
    .reduction = 1,
    .thetaMin = -160.0 / 180.0 * M_PI,
    .thetaMax = 160.0 / 180.0 * M_PI,
    .isMoving = 0,
};

Motor motor2 = {
    // Y
    .name = "motor2",
    .stepPort = GPIOB,     // D3-PB3
    .stepPin = GPIO_PIN_3, // D3-PB3
    .dirPort = GPIOB,      // D6-PB10
    .dirPin = GPIO_PIN_10, // D6-PB10
    .dir = CCW,
    .reduction = 2,
    .thetaMin = -100.0 / 180.0 * M_PI,
    .thetaMax = 100.0 / 180.0 * M_PI,
    .isMoving = 0,
};

Motor motorz = {
    .name = "motorz",
    .stepPort = GPIOB,     // D4-PB5
    .stepPin = GPIO_PIN_5, // D4-PB5
    .dirPort = GPIOA,      // D7-PA8
    .dirPin = GPIO_PIN_8,  // D7-PA8
    .dir = CCW,
    .reduction = 1,
    .thetaMin = 0,   // contacting bottom Limit SW
    .thetaMax = 100, // contacting top limit SW
    .isMoving = 0,
};

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
    GPIO_InitStruct.Pin = motor.dirPin;
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
    TIM2_Init();

    motor1.limitSwitch = theta1SW;
    motor2.limitSwitch = theta2SW;
    motorz.limitSwitch = thetazSW;
}

/**
 * @brief Move the specified motor.
 *
 * @param motor Motor to move
 * @param angle Angle in radians
 * @param speedRPM Speed in RPM
 * @return double
 */
double MoveByAngle(Motor *motor, double angle, double speedRPM)
{
    printf("Angle in MoveByAngle: %f\n", angle);
    if (angle > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        angle = angle * -1;
    }
    motor->stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV * motor->reduction);

    float timePerStep = 60.0 / (speedRPM * STEPS_PER_REV);              // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds

    motor->isMoving = 1;
    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim3);
    }
    else if (motor->name == motor2.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim4);
    }

    double angleToComplete = motor->stepsToComplete / STEPS_PER_REV / motor->reduction * 2 * M_PI;
    if (motor->dir == CW)
    {
        angleToComplete = angleToComplete * -1;
    }

    return angleToComplete;
}

double MoveByDist(Motor *motor, double dist, double speedRPM)
{
    printf("In MoveByDist function\n\r");
    if (dist > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        dist = dist * -1;
    }
    // printf("Input distance: %f\n", dist);
    // double linear_dist = dist / M_PI; // calculating lin dist travelled by rack
    // printf("Linear distance: %f\n", linear_dist);
    // float rev = linear_dist / (2 * M_PI);
    // motor->stepsToComplete = (uint32_t)((rev * Z_STEPS_PER_REV * motor->reduction));
    // printf("Steps To Complete: %.2f\n", motor->stepsToComplete);

    double theta = dist / (M_PI * M_PI);
    // motor->stepsToComplete = (uint32_t)((theta/))
    motor->stepsToComplete = (uint32_t)((theta / (2 * M_PI)) * STEPS_PER_REV); // add 20 back in

    float timePerStep = 60.0 / (speedRPM * STEPS_PER_REV);              // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds
    motor->isMoving = 1;

    if (motor->name == motorz.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim2, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim2);
    }

    return dist;
}

void StepMotor(Motor *motor)
{
    // IsMoving will be set to 0 if a limit switch is engaged
    if (!motor->stepsToComplete || !motor->isMoving)
    {
        if (motor->name == motor1.name)
        {
            HAL_TIM_Base_Stop_IT(&htim3);
        }
        else if (motor->name == motor2.name)
        {
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        else if (motor->name == motorz.name)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
        }
        motor->isMoving = 0;
    }
    HAL_GPIO_TogglePin(motor->stepPort, motor->stepPin);
    motor->stepsToComplete--;
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
            StepMotor(&motor1);
        }
    }
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
            StepMotor(&motor2);
        }
    }
}

static void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            StepMotor(&motorz);
        }
    }
}

/**
 * @brief Will stop all motors immediately.
 *
 */
void StopMotors(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);
    HAL_TIM_Base_Stop_IT(&htim2);
}

/**
 * @brief Homes the motors.
 *
 */
void HomeMotors(void)
{
    printf("Homing...\n\r");
    updateStateMachine("Homing");

    // Move positive until we hit a limit switch
    MoveByDist(&motorz, -25, 5);
    MoveByAngle(&motor1, 2 * M_PI, 5);
    MoveByAngle(&motor2, 2 * M_PI, 5);

    while (motor1.isMoving || motor2.isMoving || motorz.isMoving)
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move back 6 degrees
    // double distz = MoveByDist(&motorz, 5, 1);
    double distZ = MoveByDist(&motorz, 6.0, 1);
    double theta1 = MoveByAngle(&motor1, -6.0 / 180.0 * M_PI, 1);
    double theta2 = MoveByAngle(&motor2, -6.0 / 180.0 * M_PI, 1);

    while (motor1.isMoving || motor2.isMoving || motorz.isMoving)
    {
        HAL_Delay(1);
    }

    // Update the state machine
    updateStateMachine("Auto Wait");
    state.theta1 = motor1.thetaMax + theta1;
    state.theta2 = motor2.thetaMax + theta2;
    state.currentZ = motorz.thetaMin + distZ;
    CalculateCartesianCoords(state.theta1, state.theta2, &state.x, &state.y);
}