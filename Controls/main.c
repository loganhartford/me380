
#include "main.h"
#include "stdbool.h"
#include "math.h"
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

void delay (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0); //set counter to 0
	while(__HAL_TIM_GET_COUNTER(&htim2) < us); //wait for counter to reach entered value
}

// understand ISR and make it do anything - use it to run motors - timer will run forever, and it needs to call functions
#define STEPS_PER_REV 3200
#define anglePerStep 0.1125 //360/3200 = 0.1125
#define CW 0
#define CCW 1

// Motor struct definition
typedef struct {
    int port;           // Port of the motor
    int stepPin;        // Pin for stepping
    int directionPin;   // Pin to set direction
    int llsPort;        // Lower limit switch port
    int llsPin;         // Lower limit switch pin
    int ulsPort;        // Upper limit switch port
    int ulsPin;         // Upper limit switch pin
    bool llsTripped;    // State of lower limit switch
    bool ulsTripped;    // State of upper limit switch
    float currentAngle; // Current angle of the motor
    float targetAngle;  // Target angle for the motor to move to
    int rpm;            // Rotation speed in RPM
    int direction;      // Direction of movement
    bool moveDone;      // Flag to indicate if the move is done
} Motor;

// Motor instances with improved readability and logic
Motor motor1 = {
    .port = GPIOC,
    .stepPin = GPIO_PIN_0,
    .directionPin = GPIO_PIN_1,
    .llsPort = GPIOB,
    .llsPin = GPIO_PIN_0,
    .ulsPort = GPIOB,
    .ulsPin = GPIO_PIN_1,
    .llsTripped = true,
    .ulsTripped = true,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 50,
    .direction = CCW,
    .moveDone = false
};

Motor motor2 = {
    .port = GPIOC,
    .stepPin = GPIO_PIN_2,
    .directionPin = GPIO_PIN_3,
    .llsPort = GPIOB,
    .llsPin = GPIO_PIN_2,
    .ulsPort = GPIOB,
    .ulsPin = GPIO_PIN_4,
    .llsTripped = true,
    .ulsTripped = true,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 50,
    .direction = CCW,
    .moveDone = false
};

Motor motor3 = {
    .port = GPIOC,
    .stepPin = GPIO_PIN_4,
    .directionPin = GPIO_PIN_5,
    .llsPort = GPIOB,
    .llsPin = GPIO_PIN_5,
    .ulsPort = GPIOB,
    .ulsPin = GPIO_PIN_6,
    .llsTripped = true,
    .ulsTripped = true,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
    .rpm = 200,
    .direction = CCW,
    .moveDone = false
};


void goToAngle(Motor motor1, Motor motor2, Motor motor3)
{
	motor1.direction = CCW;
	motor2.direction = CCW;
	motor3.direction = CCW;

	if(motor1.targetAngle - motor1.currentAngle > 0) //now every motor knows its direction
	{
		motor1.direction = CW;
	}
	if(motor2.targetAngle - motor2.currentAngle > 0)
	{
		motor2.direction = CW;
	}
	if(motor3.targetAngle - motor3.currentAngle > 0)
	{
		motor3.direction = CW;
	}

  //set the direction of the motors
	HAL_GPIO_WritePin(motor1.port, motor1.directionPin, motor1.direction); 
	HAL_GPIO_WritePin(motor2.port, motor2.directionPin, motor2.direction);
	HAL_GPIO_WritePin(motor3.port, motor3.directionPin, motor3.direction);

	motor1.moveDone = false;
	motor2.moveDone = false;
	motor3.moveDone = false;

	//loop checks if motor needs to move, if all are done function will be finished its job
	while(motor1.moveDone == false || motor2.moveDone == false || motor3.moveDone == false)
	{
		//if the motor is within 0.12 degrees of the target angle it has arrived to its destination
		if(fabs(motor1.targetAngle - motor1.currentAngle) < 0.12)
		{
			motor1.moveDone = true;
		}
		else
		{
			HAL_GPIO_TogglePin(motor1.port, motor1.stepPin);
      stepperSetRpm(motor1.rpm);
      if(motor2.direction == CCW)
      {
        motor2.currentAngle -= anglePerStep/2;
      }
      else
      {
        motor2.currentAngle += anglePerStep/2;
      }
	  }
//------------------------------------------------------------------------------
		if(fabs(motor2.targetAngle - motor2.currentAngle) < 0.12)
			{
				motor2.moveDone = true;
			}
			else
			{
					HAL_GPIO_TogglePin(motor2.port, motor2.stepPin);
					stepperSetRpm(motor2.rpm);
					if(motor2.direction == CCW)
					{
						motor2.currentAngle -= anglePerStep/2;
					}
					else
					{
						motor2.currentAngle += anglePerStep/2;
					}
			}
//------------------------------------------------------------------------------
		if(fabs(motor3.targetAngle - motor3.currentAngle) < 0.12)
			{
				motor3.moveDone = true;
			}
			else
			{
					HAL_GPIO_TogglePin(motor3.port, motor3.stepPin);
					stepperSetRpm(motor3.rpm);
					if(motor3.direction == CCW)
					{
						motor3.currentAngle -= anglePerStep/2;
					}
					else
					{
						motor3.currentAngle += anglePerStep/2;
					}
			}
	}
	return;
}

void goToHome(Motor motor)
{

	HAL_GPIO_WritePin(motor.port, motor.directionPin, CCW); //set the direction of the motor

	motor.llsTripped = 1;
//while the limit switch is not tripped go CCW until it is and then stop
	while(motor.llsTripped != lsTripped)
	{
		motor.llsTripped = HAL_GPIO_ReadPin(motor.llsPort, motor.llsPin);//when pin is triggered it returns 0
		if(motor.llsTripped != lsTripped)
		{
			HAL_GPIO_TogglePin(motor.port, motor.stepPin);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);//led for test
			stepperSetRpm(motor.rpm);
		}
	}
	motor.currentAngle = 0;
	return;

}
//make function called go to home, will go CCW until limit switch is triggered


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  // MX_USART2_UART_Init();
  // MX_TIM2_Init();
  // HAL_TIM_Base_Start(&htim2);


  while (1)
  {


//code here is to try  things
	  motor1.currentAngle = 230;
	  motor1.targetAngle = 0;
	  motor1.rpm = 50;
	  motor2.currentAngle = 0;
	  motor2.targetAngle = 200;
	  motor2.rpm = 100;
	  motor3.currentAngle = 0;
	  motor3.targetAngle = 200;
	  motor3.rpm = 100;
	  goToAngle(motor1, motor2, motor3);
	  HAL_Delay(2000);

  }
}

/**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   RCC_OscInitStruct.PLL.PLLM = 8;
//   RCC_OscInitStruct.PLL.PLLN = 180;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//   RCC_OscInitStruct.PLL.PLLQ = 2;
//   RCC_OscInitStruct.PLL.PLLR = 2;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Activate the Over-Drive mode
//   */
//   if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief TIM2 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_TIM2_Init(void)
// {

//   /* USER CODE BEGIN TIM2_Init 0 */

//   /* USER CODE END TIM2_Init 0 */

//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};
//   TIM_OC_InitTypeDef sConfigOC = {0};

//   /* USER CODE BEGIN TIM2_Init 1 */

//   /* USER CODE END TIM2_Init 1 */
//   htim2.Instance = TIM2;
//   htim2.Init.Prescaler = 72-1;
//   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim2.Init.Period = 4294967295;
//   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = 0;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN TIM2_Init 2 */
// //--------------- ISR is the function to call when timer executes - interrupt service routine - when interrupt - call a function
//   /* USER CODE END TIM2_Init 2 */
//   HAL_TIM_MspPostInit(&htim2);

// }

// /**
//   * @brief USART2 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_USART2_UART_Init(void)
// {

//   /* USER CODE BEGIN USART2_Init 0 */

//   /* USER CODE END USART2_Init 0 */

//   /* USER CODE BEGIN USART2_Init 1 */

//   /* USER CODE END USART2_Init 1 */
//   huart2.Instance = USART2;
//   huart2.Init.BaudRate = 115200;
//   huart2.Init.WordLength = UART_WORDLENGTH_8B;
//   huart2.Init.StopBits = UART_STOPBITS_1;
//   huart2.Init.Parity = UART_PARITY_NONE;
//   huart2.Init.Mode = UART_MODE_TX_RX;
//   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//   if (HAL_UART_Init(&huart2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART2_Init 2 */

//   /* USER CODE END USART2_Init 2 */

// }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB4
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
