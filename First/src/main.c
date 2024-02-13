/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <main.h>

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0); // set counter to 0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us)
        ; // wait for counter to reach entered value
}
// understand ISR and make it do anything - use it to run motors - timer will run forever, and it needs to call functions
#define stepsperrev 400

void stepper_set_rpm(int rpm)
{
    delay(60000000 / stepsperrev / rpm); // set rpm, max is 3000??
} // 60 million microseconds in a minute

void stepper_drive(int step)
{
    switch (step)
    {
    case 0:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        break;
    case 4:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        break;
    case 5:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
        break;
    case 6:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
        break;
    case 7:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
        break;
    }
}

int stepCounter = 0;

void stepper_run_time(int time)
{
    for (int timeInMilliseconds; timeInMilliseconds < time; timeInMilliseconds++)
    {
        if (timeInMilliseconds == time)
        {
            stepper_set_rpm(0);
        }
    }
}

void stepper_step_angle(float angle, int direction, int rpm)
{
    float angleperstep = 360 / 400;
    int numberofssteps = (int)(angle / angleperstep);

    for (int seq = 0; seq < numberofssteps; seq++)
    {
        if (direction == 0) // CW
        {
            for (int step = 7; step >= 0; step--)
            {
                stepper_drive(step);
                stepper_set_rpm(rpm);
            }
        }
        else if (direction == 1)
        {
            for (int step = 0; step < 8; step++)
            {
                stepper_drive(step);
                stepper_set_rpm(rpm);
                stepCounter = stepcounter + 1;
            }
        }
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim2);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,0);
        //	  HAL_Delay(2000);
        //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,1);
        //	  HAL_Delay(2000);
        //	  for(int i = 0; i < 200; i++)
        //	  {
        //	  	for(int j = 0; j < 4; j ++)
        //	  	{
        //	  		stepper_drive(j);
        //	  		stepper_set_rpm(60);
        //	  	}
        //
        //	  }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        stepper_step_angle(90, 0, 10);
        HAL_Delay(1000);
        stepper_run_time(50000000); //--------------------------------------------
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
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
    htim2.Init.Prescaler = 72 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    //--------------- ISR is the function to call when timer executes - interrupt service routine - when interrupt - call a function
    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
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

#ifdef USE_FULL_ASSERT
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
