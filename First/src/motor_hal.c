#include "motor_hal.h"
#include "controls.h"
#include "limit_switch_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
static void TIM4_Init(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

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
    // .thetaMin = -100.0 / 180.0 * M_PI,
    // .thetaMax = 100.0 / 180.0 * M_PI,
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

    motor1.limitSwitch = theta1SW;
    motor2.limitSwitch = theta2SW;
    motorz.limitSwitch = thetazSW;
}

double MoveByAngle(Motor *motor, double angle, double speedRPM)
{
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
        printf("moving motor 1\n\r");
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim3);
    }
    else if (motor->name == motor2.name)
    {
        printf("moving motor 2\n\r");
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

void StepMotor(Motor *motor)
{
    HAL_GPIO_TogglePin(motor->stepPort, motor->stepPin);
    motor->stepsToComplete--;

    // IsMoving will be set to 0 if a limit switch is engaged
    if (!motor->stepsToComplete || !motor->isMoving)
    {
        if (motor->name == motor1.name)
        {
            printf("Stopping motor 1");
            HAL_TIM_Base_Stop_IT(&htim3);
        }
        else if (motor->name == motor2.name)
        {
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        motor->isMoving = 0;
    }
}

// void MoveTheta1(double angle, double speedRPM)
// {
//     if (angle > 0)
//     {
//         HAL_GPIO_WritePin(motor1.dirPort, motor1.dirPin, CCW);
//         motor1.dir = CCW;
//     }
//     else
//     {
//         HAL_GPIO_WritePin(motor1.dirPort, motor1.dirPin, CW);
//         motor1.dir = CW;
//         angle = angle * -1;
//     }
//     motor1.stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV * motor1.reduction);

//     float timePerStep = 60.0 / (speedRPM * STEPS_PER_REV);              // Time per step in seconds
//     uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds

//     __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
//     HAL_TIM_Base_Start_IT(&htim3);
// }

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
            // HAL_GPIO_TogglePin(motor1.stepPort, motor1.stepPin);
            // motor1.stepsToComplete--;
            // // If the steps are complete or if the positive limit swithc is hit and moving CCW and vice versa
            // if (!motor1.stepsToComplete || (motor1.dir && theta1SW.Pin_p_state) || (!motor1.dir && theta1SW.Pin_n_state))
            // {
            //     HAL_TIM_Base_Stop_IT(&htim3);
            //     motor1.isMoving = 0;
            // }
            // print("\n\r");
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
            // HAL_GPIO_TogglePin(motor2.stepPort, motor2.stepPin);
            // motor2.stepsToComplete--;
            // // If the steps are complete or if the positive limit swithc is hit and moving CCW and vice versa
            // if (!motor2.stepsToComplete || (motor2.dir && theta2SW.Pin_p_state) || (!motor2.dir && theta2SW.Pin_n_state))
            // {
            //     HAL_TIM_Base_Stop_IT(&htim4);
            //     motor2.isMoving = 0;
            // }
            StepMotor(&motor2);
        }
    }
}

void StopMotors(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);
}

void HomeMotors(void)
{
    printf("Homing\n\r");
    MoveByAngle(&motor1, 2 * M_PI, 5);
    MoveByAngle(&motor2, 2 * M_PI, 5);
    // MoveTheta1(2 * M_PI, 5);
    // MoveTheta2(2 * M_PI, 5);
    
    while (motor1.isMoving || motor2.isMoving)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, motor1.isMoving);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, motor2.isMoving);
        
        // Hypothetically we shouldn't have to pole in the loop here but something is wierd, should ivestigate further later.
        // theta1SW.Pin_p_state = HAL_GPIO_ReadPin(theta1SW.port, theta1SW.Pin_p);
        // theta2SW.Pin_p_state = HAL_GPIO_ReadPin(theta2SW.port, theta2SW.Pin_p);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
    HAL_Delay(2000);
    MoveByAngle(&motor1, -6.0 / 180.0 * M_PI, 1);
    MoveByAngle(&motor2, -6.0 / 180.0 * M_PI, 1);
    // MoveTheta1(-6 / 180 * M_PI, 1);
    // MoveTheta2(-6 / 180 * M_PI, 1);
    while (theta1SW.Pin_p_state || theta2SW.Pin_p_state)
    {
        // Hypothetically we shouldn't have to pole in the loop here but something is wierd, should ivestigate further later.
        theta1SW.Pin_p_state = HAL_GPIO_ReadPin(theta1SW.port, theta1SW.Pin_p);
        theta2SW.Pin_p_state = HAL_GPIO_ReadPin(theta2SW.port, theta2SW.Pin_p);
    }
    state.homed = 1;
    // StopMotors();
}