#include "main.h"
#include "controls.h"
#include "hmi_hal.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"
#include "stdbool.h"

#define DEBUG                // Enables serial print statements
#define INPUT_BUFFER_SIZE 32 // Serial reads

UART_HandleTypeDef UartHandle;
UART_HandleTypeDef huart2;

struct stateMachine state = {0};

static void SystemClockConfig(void);
void Serial_Init(void);
double ReceiveFloat(void);
void RecieveCoordinates(double *x, double *y, double *z);
void SerialDemo(void);
void DevSerialDemo(void);
void performTest(void);
void SystemHealthCheck(void);

int main(void)
{
  HAL_Init();

  SystemClockConfig();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  Serial_Init();
  Motors_Init();
  Limit_Switch_Init();
  HMI_Init();

  updateStateMachine("Unhomed");

  SystemHealthCheck();

  // Wait for the home button to be pushed
  printf("Waiting to home...\n\r");

  // Double check the maping and polarity of potentiometers
  while (1)
  {
    readAndFilter(&xPot);
    readAndFilter(&yPot);
    readAndFilter(&zPot);

    printf("xPot %f\r\n", xPot.filtered);
    printf("yPot: %f\r\n", yPot.filtered);
    printf("zPot: %f\r\n", zPot.filtered);
    double zPos = zPot.slope * zPot.filtered + zPot.b;
    printf("zPos: %f\r\n", zPos);

    GPIO_PinState gripButtonState = HAL_GPIO_ReadPin(gripButton.port, gripButton.pin);
    if (gripButtonState)
    {
      printf("Switch is high\r\n");
    }
    else
    {
      printf("Switch is low\r\n");
    }
    printf("\r\n");

    HAL_Delay(1000); // Example delay, adjust as needed
  }

  while (HAL_GPIO_ReadPin(homeButton.port, homeButton.pin))
  {
    HAL_Delay(1);
  }

  // Home the robot
  HomeMotors();

  // Default to auto-wait, where user can either perform the test or switch to manual
  while (1)
  {
    if (HAL_GPIO_ReadPin(runTestButton.port, runTestButton.pin) == GPIO_PIN_RESET)
    {
      updateStateMachine("Auto Move");
      performTest();
      updateStateMachine("Auto Wait");
    }
    else if (HAL_GPIO_ReadPin(autoManButton.port, autoManButton.pin) == GPIO_PIN_RESET)
    {
      printf("Switched to Manual Mode (Serial Demo)\n\r");

      // Set z motor to slider position
      double zPos = zPot.slope * zPot.value + zPot.b;
      MoveToZ(zPos, 35.0);
      HAL_Delay(500); // So button isn't "double-pressed"
      while (motorz.isMoving)
      {
        HAL_Delay(1);
      }

      updateStateMachine("Manual");

      Manual_Mode();

      // SerialDemo();   // To be replaced w/ manual mode
      printf("Switched to Automatic Mode\n\r");
      updateStateMachine("Auto Wait");
    }
    HAL_Delay(1);
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
  updateStateMachine("Faulted");
  while (1)
  {
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
void RecieveCoordinates(double *x, double *y, double *z)
{
  printf("Enter in desired X coordinate: \n\r");
  *x = ReceiveFloat();
  printf("Enter in desired Y corrdinate: \n\r");
  *y = ReceiveFloat();
  printf("Enter in desired Z coordinate: \n\r");
  *z = ReceiveFloat();
}

/**
 * @brief Runs a demo which allows the user to send the robot x and y position commands and move the motors.
 *
 */
void SerialDemo(void)
{
  PrintState();
  while (1)
  {
    if (HAL_GPIO_ReadPin(runTestButton.port, runTestButton.pin) == GPIO_PIN_RESET)
    {
      double x, y, z;
      RecieveCoordinates(&x, &y, &z);
      printf("Moving to: ");
      PrintCaresianCoords(x, y);
      MoveTo(x, y, 10.0);
      MoveToZ(z, 35.0);
      printf("\n\r");
    }
    else if (HAL_GPIO_ReadPin(autoManButton.port, autoManButton.pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(500); // So button isn't "double-pressed"
      return;
    }
    HAL_Delay(1);
  }
}

/**
 * @brief Alternative ReceieveCoordinates for dev purposes
 *
 * @param x
 * @param y
 * @param z
 */
void DevRecieveCoordinates(double *x, double *y, double *z)
{
  // printf("Enter in desired X coordinate: \n\r");
  // *x = ReceiveFloat();
  // printf("Enter in desired Y corrdinate: \n\r");
  // *y = ReceiveFloat();
  printf("Enter in desired Z coordinate: \n\r");
  *z = ReceiveFloat();
}

/**
 * @brief Alternative serial demo for dev purposes
 *
 */
void DevSerialDemo(void)
{
  double x, y, z;
  DevRecieveCoordinates(&x, &y, &z);
  // printf("Moving to: ");
  // PrintCaresianCoords(x, y);
  // MoveTo(x, y, 10.0);
  MoveToZ(z, 35.0);
  printf("\n\r");
}

/**
 * @brief Runs the automatic test as specified in the course outline
 *
 */
void performTest(void)
{
  HAL_Delay(1000);

  // What is should be
  // double xStart = 0, yStart = 202.5, xEnd = 175, yEnd = -122.5, zUp = 2, zDown = 88;
  // What actually gets us there
  double xStart = -4, yStart = 198, zUp = 5;
  double xEnd = 160, yEnd = -120, zDown = 90;

  // Moving to Start Location (M1 & M2 Active)
  printf("Moving to start\n\r");
  MoveTo(xStart, yStart, 10.0);
  MoveToZ(zDown, 25.0);
  HAL_Delay(1500);
  gripperOpen(&gripper);
  while (motor1.isMoving || motor2.isMoving)
  {
    HAL_Delay(1);
  }
  // Moving Rack Down (MZ Active)

  while (motorz.isMoving)
  {
    HAL_Delay(1);
  }
  // gripper should actuate here
  HAL_Delay(1000);

  gripperClose(&gripper);
  HAL_Delay(1000);
  // Moving Rack Back Up (MZ Active)
  MoveToZ(zUp, 25.0);
  while (motorz.isMoving)
  {
    HAL_Delay(1);
  }
  HAL_Delay(1000);

  // Moving to End Location (M1 & M2 Active)
  printf("Moving to End\n\r");
  MoveTo(xEnd, yEnd, 5.0);
  while (motor1.isMoving || motor2.isMoving)
  {
    HAL_Delay(1);
  }

  // Moving Rack Down (MZ Active)
  MoveToZ(zUp + 15, 25.0);
  while (motorz.isMoving)
  {
    HAL_Delay(1);
  }

  HAL_Delay(1000);
  gripperOpen(&gripper);
  HAL_Delay(1000);

  // Move of the dice
  MoveTo(xEnd + 70, yEnd, 10.0);
  while (motor1.isMoving || motor2.isMoving)
  {
    HAL_Delay(1);
  }
  MoveTo(xEnd + 70, yEnd + 70, 10.0);
  while (motor1.isMoving || motor2.isMoving)
  {
    HAL_Delay(1);
  }
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
  else if (!homeButton.pin_state)
  {
    printf("Error: Check home button\n\r");
  }
  else if (!runTestButton.pin_state)
  {
    printf("Error: Check runTest button\n\r");
  }
  else if (!autoManButton.pin_state)
  {
    printf("Error: Check autoMan button\n\r");
  }
  else
  {
    return;
  }
  ErrorHandler();
}