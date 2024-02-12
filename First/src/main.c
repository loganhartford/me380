#include "main.h"
#include "controls.h"

#define DEBUG // Enables serial print statements

UART_HandleTypeDef UartHandle;

struct stateMachine state = {0}; // State machine

static void SystemClockConfig(void);
static void ErrorHandler(void);
void SerialInit(void);

// Motor Stuff
#include "stm32f4xx_hal_tim.h"
TIM_HandleTypeDef htim2;
 // this should be public

static void MX_TIM2_Init(void);

#define STEPS_PER_REV 3200
#define ANGLE_PER_STEP 0.1125 //360/3200 = 0.1125
#define CW 0
#define CCW 1

typedef struct {
    GPIO_TypeDef* stepPort; // Port of the motor
    uint16_t stepPin;       // Pin for stepping
    GPIO_TypeDef* dirPort;
    uint16_t directionPin;  // Pin to set direction
    float currentAngle;     // Current angle of the motor
    float targetAngle;      // Target angle for the motor to move to
    int rpm;                // Rotation speed in RPM
    int direction;          // Direction of movement
    bool moveDone;          // Flag to indicate if the move is done
} Motor;

// Motor instances with improved readability and logic
Motor motor_x = {
    .stepPort = GPIOD,          // D7-PA8
    .stepPin = GPIO_PIN_2,      // D7-PA8
    .dirPort = GPIOB,           // D5-PB4
    .directionPin = GPIO_PIN_4, // D5-PB4
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 50,
    .direction = CCW,
    .moveDone = false
};

Motor motor_y = {
    .stepPort = GPIOB,          // D3-PB3
    .stepPin = GPIO_PIN_3,      // D3-PB3
    .dirPort = GPIOB,           // D6-PB10
    .directionPin = GPIO_PIN_10,// D6-PB10
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 50,
    .direction = CCW,
    .moveDone = false
};

Motor motor_z = {
    .stepPort = GPIOB,          // D4-PB5
    .stepPin = GPIO_PIN_5,      // D4-PB5
    .dirPort = GPIOA,           // D7-PA8
    .directionPin = GPIO_PIN_8, // D7-PA8
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 200,
    .direction = CCW,
    .moveDone = false
};

void delay (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0); //set counter to 0
  // print(us);
	while(__HAL_TIM_GET_COUNTER(&htim2) < us){
    printf("%u\n\r", __HAL_TIM_GET_COUNTER(&htim2));
  }; //wait for counter to reach entered value
}

// //calculate rpm
void StepperSetRpm(int rpm)
{
	delay(60000000/STEPS_PER_REV/rpm); //set rpm
}

// Individual motor initialization function
void Motor_Init(Motor motor) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clocks
    if (motor.stepPort == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (motor.stepPort == GPIOB) {
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

// Main initialization function
void Motors_Init(void) {
    Motor_Init(motor_x);
    Motor_Init(motor_y);
    Motor_Init(motor_z);
}


void goToAngle(Motor motor_x, Motor motor_y, Motor motor_z)
{
	motor_x.direction = CCW;
	motor_y.direction = CCW;
	motor_z.direction = CCW;

	if(motor_x.targetAngle - motor_x.currentAngle > 0) //now every motor knows its direction
	{
		motor_x.direction = CW;
	}
	if(motor_y.targetAngle - motor_y.currentAngle > 0)
	{
		motor_y.direction = CW;
	}
	if(motor_z.targetAngle - motor_z.currentAngle > 0)
	{
		motor_z.direction = CW;
	}
  printf("2.1\n\f");
  //set the direction of the motors
	HAL_GPIO_WritePin(motor_x.dirPort, motor_x.directionPin, motor_x.direction); 
	HAL_GPIO_WritePin(motor_y.dirPort, motor_y.directionPin, motor_y.direction);
	HAL_GPIO_WritePin(motor_z.dirPort, motor_z.directionPin, motor_z.direction);

	motor_x.moveDone = false;
	motor_y.moveDone = false;
	motor_z.moveDone = false;

	//loop checks if motor needs to move, if all are done function will be finished its job
	while(motor_x.moveDone == false || motor_y.moveDone == false || motor_z.moveDone == false)
	{
		//if the motor is within 0.12 degrees of the target angle it has arrived to its destination
		if(fabs(motor_x.targetAngle - motor_x.currentAngle) < 0.12)
		{
			motor_x.moveDone = true;
		}
		else
		{
      printf("2.2\n\f");
			HAL_GPIO_TogglePin(motor_x.stepPort, motor_x.stepPin);
      printf("2.3\n\f");
      StepperSetRpm(motor_x.rpm);
      printf("2.4\n\f");
      if(motor_y.direction == CCW)
      {
        motor_y.currentAngle -= ANGLE_PER_STEP/2;
      }
      else
      {
        motor_y.currentAngle += ANGLE_PER_STEP/2;
      }
	  }
//------------------------------------------------------------------------------
		if(fabs(motor_y.targetAngle - motor_y.currentAngle) < 0.12)
			{
				motor_y.moveDone = true;
			}
			else
			{
					HAL_GPIO_TogglePin(motor_y.stepPort, motor_y.stepPin);
					StepperSetRpm(motor_y.rpm);
					if(motor_y.direction == CCW)
					{
						motor_y.currentAngle -= ANGLE_PER_STEP/2;
					}
					else
					{
						motor_y.currentAngle += ANGLE_PER_STEP/2;
					}
			}
//------------------------------------------------------------------------------
		if(fabs(motor_z.targetAngle - motor_z.currentAngle) < 0.12)
			{
				motor_z.moveDone = true;
			}
			else
			{
					HAL_GPIO_TogglePin(motor_z.stepPort, motor_z.stepPin);
					StepperSetRpm(motor_z.rpm);
					if(motor_z.direction == CCW)
					{
						motor_z.currentAngle -= ANGLE_PER_STEP/2;
					}
					else
					{
						motor_z.currentAngle += ANGLE_PER_STEP/2;
					}
			}
	}
	return;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    ErrorHandler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    ErrorHandler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    ErrorHandler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    ErrorHandler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    ErrorHandler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
//--------------- ISR is the function to call when timer executes - interrupt service routine - when interrupt - call a function
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}


int main(void) {
  HAL_Init();
  SystemClockConfig();
  SerialInit();
  MX_TIM2_Init();
  Motors_Init();

  // HOME THE ROBOT
  InitializeStateMachine(); // I would maybe even put this function call in the homing function

  while (1) {
    // HAL_Delay(2000);
  
    printf("1\n\r");
    motor_x.currentAngle = 230;
	  motor_x.targetAngle = 0;
	  motor_x.rpm = 50;
	  motor_y.currentAngle = 0;
	  motor_y.targetAngle = 200;
	  motor_y.rpm = 100;
	  motor_z.currentAngle = 0;
	  motor_z.targetAngle = 200;
	  motor_z.rpm = 100;
    printf("2\n\r");
	  goToAngle(motor_x, motor_y, motor_z);
    printf("3\n\r");
	  HAL_Delay(2000);

    // 1. Robot is sitting idle
    
    // - Listen for button inputs
    // - Listen for joystick inputs

    // 2. Robot is moving to some locations

    // - Executing motion
    // - Interrupt can cancel motion
    // - Motion is a blocking function for right now
  }
}

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

//

/**
 * @brief Configures the clock settings.
 * 
 */
void SystemClockConfig(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    ErrorHandler();
  }
  
   /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    ErrorHandler();
  }

}

/**
 * @brief Will hold the device in an infinte look on error.
 * 
 */
static void ErrorHandler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}

/**
 * @brief Initializes UART for printing to the serial monitor.
 * 
 */
void SerialInit(void) {
  UartHandle.Instance = USARTx;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK) {
    ErrorHandler();
  }
}

