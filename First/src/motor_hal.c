#include "motor_hal.h"

TIM_HandleTypeDef htim2;

void TIM2_Init(void);
void delay(uint32_t us);
void StepperSetRpm(int rpm);
void Motor_Init(Motor motor);

// Motor Objects
Motor motor1 = {
    // X
    .stepPort = GPIOA,          // D2-PA10
    .stepPin = GPIO_PIN_10,     // D2-PA10
    .dirPort = GPIOB,           // D5-PB4
    .directionPin = GPIO_PIN_4, // D5-PB4
    .currentAngle = 0.0,
    .targetAngle = 0.0,
    .rpm = 50.0,
    .direction = CCW,
    .moveDone = false,
    .radsPerStep = 2 * M_PI / STEPS_PER_REV,
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
    .currentAngle = 0.0,
    .targetAngle = 0.0,
    .rpm = 50.0,
    .direction = CCW,
    .moveDone = false,
    .radsPerStep = 2 * M_PI / STEPS_PER_REV,
    .reduction = 2,
    .thetaMin = -100.0 / 180.0 * M_PI,
    .thetaMax = 100.0 / 180.0 * M_PI,
};

Motor motorz = {                // Z
    .stepPort = GPIOB,          // D4-PB5
    .stepPin = GPIO_PIN_5,      // D4-PB5
    .dirPort = GPIOA,           // D7-PA8
    .directionPin = GPIO_PIN_8, // D7-PA8
    .currentAngle = 0.0,
    .targetAngle = 0.0,
    .rpm = 200.0,
    .direction = CCW,
    .moveDone = false,
    .radsPerStep = 2 * M_PI / Z_STEPS_PER_REV,
    .reduction = 1};

/**
 * @brief Sets up timer 2 as a simple counter for creating the delay used for commutating the motors.
 *
 */
void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 88; // For 1 µs tick
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF; // Max period
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        ErrorHandler();
    }

    HAL_TIM_Base_Start(&htim2);
}

/**
 * @brief Takes in a motor struct and initialized the associated pins.
 *
 * @param motor Motor struct
 */
void Motor_Init(Motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clocks
    if (motor.stepPort == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (motor.stepPort == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } // Add more conditions if using other GPIO ports

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
    TIM2_Init();
}

void StepMotor(Motor *motor)
{
    if (fabs(motor->targetAngle - motor->currentAngle) <= motor->radsPerStep)
    {
        motor->moveDone = true;
    }
    else
    {
        HAL_GPIO_TogglePin(motor->stepPort, motor->stepPin);
        StepperSetRpm(motor->rpm);
        if (motor->direction == CCW)
        {
            motor->currentAngle -= motor->radsPerStep / motor->reduction;
        }
        else
        {
            motor->currentAngle += motor->radsPerStep / motor->reduction;
        }
    }
}

/**
 * @brief Takes in an angle for each stepper motor and moves the motor and a pointer for each motor to store the motion completed after each iteration.
 *
 * @param theta1 angle to move motor 1 by.
 * @param theta2 angle to move motor 2 by.
 * @param thetaz angle to move motor z by.
 * @param realtheta1 pointer to revolutions completed by motor 1.
 * @param realtheta2 pointer to revolutions completed by motor 2.
 * @param realthetaz pointer to revolutions completed by motor z.
 */
void MoveByAngle(double theta1, double theta2, double thetaz, double *realtheta1, double *realtheta2, double *realthetaz)
{
    // 1. Calculate the number of steps for each motor
    // 2. Set the direction based on the sign of the request
    // 3. Increment the counters each iteration
    motor1.targetAngle = theta1;
    motor2.targetAngle = theta2;
    motorz.targetAngle = thetaz;

    motor1.currentAngle = 0;
    motor2.currentAngle = 0;
    motorz.currentAngle = 0;

    motor1.direction = CCW;
    motor2.direction = CCW;
    motorz.direction = CCW;

    if (motor1.targetAngle - motor1.currentAngle > 0) // now every motor knows its direction
    {
        motor1.direction = CW;
    }
    if (motor2.targetAngle - motor2.currentAngle > 0)
    {
        motor2.direction = CW;
    }
    if (motorz.targetAngle - motorz.currentAngle > 0)
    {
        motorz.direction = CW;
    }
    // set the direction of the motors
    HAL_GPIO_WritePin(motor1.dirPort, motor1.directionPin, motor1.direction);
    HAL_GPIO_WritePin(motor2.dirPort, motor2.directionPin, motor2.direction);
    HAL_GPIO_WritePin(motorz.dirPort, motorz.directionPin, motorz.direction);

    motor1.moveDone = false;
    motor2.moveDone = false;
    motorz.moveDone = false;

    // loop checks if motor needs to move, if all are done function will be finished its job
    while (motor1.moveDone == false || motor2.moveDone == false || motorz.moveDone == false)
    {
        StepMotor(&motor1);
        StepMotor(&motor2);
        StepMotor(&motorz);
    }
    // This isn't proper yet, return the actual angle deltas
    *realtheta1 = theta1;
    *realtheta2 = theta2;
    *realthetaz = thetaz;
}

/**
 * @brief Uses timer 2 to create a delay between motor commutations.
 *
 * @param us - delay in microseconds
 */
void delay(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Set counter to 0

    while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    {
    };
}

/**
 * @brief Sets the speed of the motor by calling delay.
 *
 * @param rpm desired rpm of the motor.
 */
void StepperSetRpm(int rpm)
{
    delay(60000000 / STEPS_PER_REV / rpm); // set rpm
}