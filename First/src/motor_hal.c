#include "motor_hal.h"
#include "controls.h"
#include "limit_switch_hal.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
static void TIM4_Init(void);
static void TIM7_Init(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM7_IRQHandler(void);
void gripperClose(ServoMotor *gripper);
void gripperOpen(ServoMotor *gripper);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

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
    .thetaMin = -164.0 / 180.0 * M_PI,
    .thetaMax = 164.0 / 180.0 * M_PI,
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
    .thetaMin = -110.0 / 180.0 * M_PI,
    .thetaMax = 110.0 / 180.0 * M_PI,
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

ServoMotor gripper = {
    .pwmPort = GPIOA,
    .pwmPin = GPIO_PIN_5,
    .closedPosition = 10,  // limit when gripper is closing
    .openPosition = 2      // limit when gripper is open
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
    TIM7_Init();

    MX_TIM2_Init();
    MX_ADC1_Init();

    if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
	    ErrorHandler();
    }

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

    // Gain scheduling setup
    motor->stepsToComplete = (uint32_t)((angle / (2 * M_PI)) * STEPS_PER_REV * motor->reduction);
    // Speed up for first 1/4 steps
    motor->stepsToSpeedUp = 3.0 / 4.0 * motor->stepsToComplete;
    // Slow down for last 1/4 steps
    motor->stepsToSlowDown = 1.0 / 4.0 * motor->stepsToComplete;
    // RPM delta per step
    motor->slope = (speedRPM - MIN_RPM) / (motor->stepsToSlowDown);
    // Start at the min rpm
    motor->currentRPM = MIN_RPM;

    float timePerStep = 60.0 / (MIN_RPM * STEPS_PER_REV * motor->reduction); // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1;      // Time per toggle, in microseconds

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
    double theta = dist / (M_PI);
    motor->stepsToComplete = (uint32_t)(theta * Z_STEPS_PER_REV);

    // Gain scheduling setup
    // Speed up for first 1/4 steps
    motor->stepsToSpeedUp = 3.0 / 4.0 * motor->stepsToComplete;
    // Slow down for last 1/4 steps
    motor->stepsToSlowDown = 1.0 / 4.0 * motor->stepsToComplete;
    // RPM delta per step
    motor->slope = (speedRPM - MIN_RPM) / (motor->stepsToSlowDown);
    // Start at the min rpm
    motor->currentRPM = MIN_RPM;

    float timePerStep = 60.0 / (speedRPM * Z_STEPS_PER_REV);            // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1; // Time per toggle, in microseconds
    motor->isMoving = 1;

    if (motor->name == motorz.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim7, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim7);
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
            HAL_TIM_Base_Stop_IT(&htim7);
        }
        motor->isMoving = 0;
    }
    // Adjust the speed of the motor
    if (motor->stepsToComplete > motor->stepsToSpeedUp)
    {
        motor->currentRPM += motor->slope;
    }
    else if (motor->stepsToComplete < motor->stepsToSlowDown)
    {
        motor->currentRPM -= motor->slope;
    }

    // Change the timer period based on the current rpm
    float timePerStep = 60.0 / (motor->currentRPM * STEPS_PER_REV * motor->reduction); // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1;                // Time per toggle, in microseconds

    // Set the new timer period
    if (motor->name == motor1.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
    }
    else if (motor->name == motor2.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
    }
    else if (motor->name == motorz.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim7, timerPeriod);
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

static void TIM7_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim7);

    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
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
    HAL_TIM_Base_Stop_IT(&htim7);
}

/**
 * @brief Homes the motors.
 *
 */
void HomeMotors(void)
{
    printf("Homing...\n\r");
    updateStateMachine("Homing");

    gripperClose(&gripper);
    MoveByDist(&motorz, -1000, 25);
    MoveByAngle(&motor1, 2 * M_PI, 5);
    MoveByAngle(&motor2, 2 * M_PI, 5);

    while (motor1.isMoving || motor2.isMoving || motorz.isMoving)
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move back 6 degrees
    double distZ = MoveByDist(&motorz, 10.0, 5);
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
    printf("Current Coords in x-y:");
    PrintCaresianCoords(state.x, state.y);
}

/**
 * @brief Closes the gripper
 *
 * @param gripper Gripper Object
 */
void gripperClose(ServoMotor *gripper)
{

    uint16_t raw;
    int average; // currentDraw?
    //char msg[25];
    //uint8_t buffer_uart[] = "Limit exceeded\r\n";

    raw = 0;
    average = 0;

//closing the servo and measuring the current----------------------------------------------------------------------------
    while(gripper->position <= gripper->closedPosition && average < 700)
    {
        //printf("A\n\r");
        gripper->position += 0.5;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period*gripper->position/200);
        average = 0;
        //for loop is taking 20 readings and taking average to deal with noise
        for (int j = 0; j < 20; j++)
        {
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            raw = HAL_ADC_GetValue(&hadc1);
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            average += raw;
            HAL_Delay(1);
        }        

        average = average / 20;
        //sprintf(msg, "%hu  ", average);//prints current readings and that the current limit has been exceeded
        //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 2);
            HAL_Delay(1);
    } 
    //if statement says we've hit something and that we should get out of the loop
    if(average > 700)
    {
        //HAL_UART_Transmit(&huart2, buffer_uart, sizeof(buffer_uart), 1);
        HAL_Delay(1000);
        average = 0;
    }
    gripper->isOpen = false;
}

/**
 * @brief Opens the gripper
 *
 * @param gripper Gripper Object
 */
void gripperOpen(ServoMotor *gripper)
{
    while(gripper->position >= gripper->openPosition)
    {
       gripper->position -= 0.15;
       __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period*gripper->position/200);
	     HAL_Delay(5);
    } 
    gripper->isOpen = true;
}

static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Enable clock for TIM2
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // Enable clock for GPIOA (if not already enabled)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim2.Instance = TIM2;
    // Adjust Prescaler to get a 1 MHz timer clock frequency
    htim2.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // Now directly using SystemCoreClock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999; // Sets the PWM frequency to 50 Hz (20 ms period)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2); // Initialize TIM2 in PWM mode

    // Configure the PWM channel
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500; // Initial pulse width of 1.5 ms, adjust as needed for servo positioning
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    // Configure GPIO Pin PA5 for Alternate Function (TIM2_CH1)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Push-Pull Alternate Function
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // Ensure this is the correct alternate function for your MCU series
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Start PWM on TIM2 Channel 1
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)*/
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
  HAL_ADC_MspInit(&hadc1);
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    ErrorHandler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    ErrorHandler();
  }
}