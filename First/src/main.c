#include "main.h"
#include "controls.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"

#define DEBUG                // Enables serial print statements
#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;

struct stateMachine state = {0};

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *x, double *y);
void SerialDemo(void);
void GPIO_Init(void);
void SystemHealthCheck(void);

// Limit switches

int main(void)
{
  HAL_Init();
  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  Serial_Init();
  Motors_Init();
  Limit_Switch_Init();
  GPIO_Init(); // Initialize GPIO for LED

  InitializeStateMachine();

  SystemHealthCheck();

  // Wait for the home button to be pushed
  printf("Waiting to home...\n\r");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
  {
    HAL_Delay(1);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);

  // Home the robot
  HomeMotors();

  while (1)
  {
    SerialDemo(); // This will halt execution
  }
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    ErrorHandler();
  }

  /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if (ret != HAL_OK)
  {
    while (1)
    {
      ;
    }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    ErrorHandler();
  }
}

/**
 * @brief Will hold the device in an infinte loop on error.
 *
 */
void ErrorHandler(void)
{
  while (1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
  }
}

/**
 * @brief Initializes UART for printing to the serial monitor.
 *
 */
void Serial_Init(void)
{
  UartHandle.Instance = USARTx;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    ErrorHandler();
  }
}

/**
 * @brief Halts execution until a return character is entered into the serial monitor. Tries to convert the input into a double.
 *
 * @return double - serial monitor input
 */
double ReceiveFloat(void)
{
  char inputBuffer[INPUT_BUFFER_SIZE];
  uint8_t receivedChar;
  double receivedFloat;

  memset(inputBuffer, 0, INPUT_BUFFER_SIZE); // Clear input buffer
  int bufferIndex = 0;

  while (1)
  {
    // Receive a single character
    if (HAL_UART_Receive(&UartHandle, &receivedChar, 1, 0xFFFF) == HAL_OK)
    {
      // Check for end of number input (e.g., newline character)
      if (receivedChar == '\n' || receivedChar == '\r')
      {
        inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
        break;                           // Exit the loop
      }
      else
      {
        // Store the received character into the buffer
        if (bufferIndex < INPUT_BUFFER_SIZE - 1) // Prevent buffer overflow
        {
          inputBuffer[bufferIndex++] = receivedChar;
        }
      }
    }
  }
  // Convert the received string to a floating-point number
  receivedFloat = atof(inputBuffer);

  return receivedFloat;
}

/**
 * @brief Halts program execution and asks user to input an x and a y coordinate.
 *
 * @param x pointer to x coordinate
 * @param y pointer to y cordinate
 */
void RecieveCoordinates(double *x, double *y)
{
  printf("Enter in disired X coordinate: \n\r");
  *x = ReceiveFloat();
  printf("Enter in disred Y corrdinate: \n\r");
  *y = ReceiveFloat();
}

/**
 * @brief Runs a demo which allows the user to send the robot x and y position commands and move the motors.
 *
 */
void SerialDemo(void)
{
  PrintState();

  double x, y;
  RecieveCoordinates(&x, &y);
  printf("Moving to: ");
  PrintCaresianCoords(x, y);
  MoveTo(x, y);
  printf("\n\r");
}

/**
 * @brief Initialize general purpose IOs
 *
 */
void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // LED pin configuration
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push Pull Mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0); // Off by default

  GPIO_InitTypeDef GPIO_InitStruct2 = {0};

  // LED pin configuration
  GPIO_InitStruct2.Pin = GPIO_PIN_8;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP; // Push Pull Mode
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);

  GPIO_InitTypeDef GPIO_InitStruct3 = {0};

  // Homing button
  GPIO_InitStruct3.Pin = GPIO_PIN_9;
  GPIO_InitStruct3.Mode = GPIO_MODE_INPUT; // Push Pull Mode
  GPIO_InitStruct3.Pull = GPIO_NOPULL;
  GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct3);
}

/**
 * @brief Put all system health checks here
 *
 */
void SystemHealthCheck(void)
{
  // Check that all limit switches are closed (NC switched).
  if (theta1SW.Pin_p_state)
  {
    printf("Error: check theta1+ sw\n\r");
  }
  else if (theta1SW.Pin_n_state)
  {
    printf("Error: check theta1- sw\n\r");
  }
  else if (theta2SW.Pin_p_state)
  {
    printf("Error: check theta2+ sw\n\r");
  }
  else if (theta2SW.Pin_n_state)
  {
    printf("Error: check theta2- sw\n\r");
  }
  else if (thetazSW.Pin_p_state)
  {
    printf("Error: check thetaz+ sw\n\r");
  }
  else if (thetazSW.Pin_n_state)
  {
    printf("Error: check thetaz- sw\n\r");
  }
  else
  {
    return;
  }
  ErrorHandler();
}