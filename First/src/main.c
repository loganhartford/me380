#include "main.h"

#define DEBUG // Enables serial print statements

UART_HandleTypeDef UartHandle;

static void SystemClockConfig(void);
static void ErrorHandler(void);
void SerialInit(void);


void InitializeStateMachine(void);
void CalculateJointAngle(double x, double y, double solns[2][2]);
void PrintAnglesInDegrees(double theta1, double theta2);
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y);
double CalculateQuickestPath(double cur_theta, double targ_theta);
double CalculateMotorDelta(double delta);
void MoveTo(double x, double y);
void MoveBy(double rel_x, double rel_y);


#include <math.h>
#include <stdbool.h>

// Robot Parameters
#define LINK_1 85.0     // mm
#define LINK_2 190.0    // mm

#define DEGREES_PER_STEP (M_PI / 100.0) // Converted 1.8 degrees to radians
#define M1_GEAR_REDUCTION 2
#define M2_GEAR_REDUCTION 2
#define RESOLUTION (DEGREES_PER_STEP / M1_GEAR_REDUCTION)

#define HOMED_THETA1 0.0
#define HOMED_THETA2 0.0
#define HOMED_THETA1 CalculateMotorDelta(120 * 180 / M_PI)
#define HOMED_THETA2 CalculateMotorDelta(M_PI)

struct stateMachine {
  bool homed;
  bool inmotion;
  double theta1;
  double theta2;
  double x;
  double y;
};

struct stateMachine state = {0};

int main(void) {
  HAL_Init();
  SystemClockConfig();
  SerialInit();
  InitializeStateMachine();
  
  double x;
  double y;
  double solns[2][2];
  while (1) {
    HAL_Delay(2000);
    printf("\n\r");
    
    // CalculateCartesianCoords(-122.925*M_PI/180, 162.61*M_PI/180, &x, &y);
    // CalculateCartesianCoords(198.44*M_PI/180, -87.03*M_PI/180, &x, &y);
    // CalculateCartesianCoords(240.33*M_PI/180, 78.07*M_PI/180, &x, &y);
    double tempx = -150;
    double tempy = 150;
    CalculateJointAngle(tempx, tempy, solns);
    printf("\n\r");
    state.theta1 = 0;
    state.theta2 = 0;
    moveTo(tempx, tempy);

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

/**
 * @brief Converts cartesian coordinates to joint angles of the robot.
 * 
 * @param x coordinate.
 * @param y coordinate.
 * @param solns An array which will contain the set(2) of possible angles.
 */
void CalculateJointAngle(double x, double y, double solns[2][2]) {
    // Calculate length of end effector vector
    double R = sqrt(x * x + y * y);

    // Angle of vector using atan2 to handle quadrants
    double THETA = atan2(y, x);

    // Handle the limits of acos
    double acosarg = (R * R - LINK_1 * LINK_1 - LINK_2 * LINK_2) / (-2 * LINK_1 * LINK_2);
    double beta;
    if (acosarg < -1.0) {
        beta = M_PI;
    } else if (acosarg > 1.0) {
        beta = 0.0;
    } else {
        beta = acos(acosarg);
    }

    double alpha;
    double break_r = sqrt(LINK_2 * LINK_2 - LINK_1 * LINK_1);
    if (R > break_r) {
        alpha = asin((LINK_2 * sin(beta)) / R);
    } else if (R > 0.0) {
        alpha = M_PI / 2 + (M_PI / 2 - asin((LINK_2 * sin(beta)) / R));
    } else {
        alpha = 0.0;
    }

    // Assembly both possible solutions [theta1, theta2]
    solns[0][0] = THETA - alpha;
    solns[0][1] = M_PI - beta;
    solns[1][0] = THETA + alpha;
    solns[1][1] = beta - M_PI;

    // Keep angles within [-360, 360]
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            if (solns[i][j] > 2 * M_PI) {
                solns[i][j] -= 2 * M_PI;
            } else if (solns[i][j] < -2 * M_PI) {
                solns[i][j] += 2 * M_PI;
            }
        }
    }

  #ifdef DEBUG
    PrintAnglesInDegrees(solns[0][0], solns[0][1]);
    PrintAnglesInDegrees(solns[1][0], solns[1][1]);
  #endif
}

/**
 * @brief Helper function which prints two radian angles in degrees to the serial monitor.
 * 
 * @param theta1 Radian angle 1.
 * @param theta2 Radian angle 2.
 */
void PrintAnglesInDegrees(double theta1, double theta2) {
     theta1 = theta1 * (180.0 / M_PI);
     theta2 = theta2 * (180.0 / M_PI);

    int int_part = (int)theta1;
    int decimal_part = abs((int)((theta1 - int_part) * 100)); // 2 decimal places
    int int_part2 = (int)theta2;
    int decimal_part2 = abs((int)((theta2 - int_part2) * 100)); // 2 decimal places

    printf("(%d.%d degrees, %d.%d degrees)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Prints cartesion coordinates to the serial monitor.
 * 
 * @param x coordinate.
 * @param y coordinate.
 */
void PrintCaresianCoords(double x, double y) {
  int int_part = (int)x;
  int decimal_part = abs((int)((x - int_part) * 100)); // 2 decimal places
  int int_part2 = (int)y;
  int decimal_part2 = abs((int)((y - int_part2) * 100)); // 2 decimal places

  printf("(%d.%d, %d.%d)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Sets the state machine to it's default state.
 * 
 * Should be called after the robot is homed.
 * 
 */
void InitializeStateMachine(void) {
  state.homed = 0;
  state.inmotion = 0;
  state.theta1 = HOMED_THETA1;
  state.theta2 = HOMED_THETA2;
  CalculateCartesianCoords(HOMED_THETA1, HOMED_THETA2, &state.x, &state.y);
}

/**
 * @brief Converts cartesion coordinates to link angles of the robot.
 * 
 * @param theta1 angle of link 1.
 * @param theta2 angle of link 2/
 * @param x position
 * @param y position
 */
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y) {
    *x = cos(theta1) * LINK_1 + cos(theta1 + theta2) * LINK_2;
    *y = sin(theta1) * LINK_1 + sin(theta1 + theta2) * LINK_2;

    #ifdef DEBUG
      PrintCaresianCoords(*x,*y);
    #endif
}

/**
 * @brief Caclculates the smallest absolute able between two angles.
 * 
 * @param cur_theta current angle.
 * @param targ_theta desired angle.
 * @return double - the smallest delta to get from the current to the desired angle.
 */
double CalculateQuickestPath(double cur_theta, double targ_theta) {
    double delta = targ_theta - cur_theta;
    if (delta > M_PI) {
        return delta - 2 * M_PI;
    } else if (delta < -M_PI) {
        return delta + 2 * M_PI;
    }
    return delta;
}

/**
 * @brief Converts an angle to an acceptable angle for the motors.
 * 
 * @param delta angle you want to move the motor by.
 * @return double - the motor acceptable angle.
 */
double CalculateMotorDelta(double delta) {
    return round(delta / RESOLUTION) * RESOLUTION;
}

/**
 * @brief Move the robot to the x and y coordinates.
 * 
 * @param x coordniate.
 * @param y coordinate.
 */
void moveTo(double x, double y) {
    double solns[2][2];
    CalculateJointAngle(x, y, solns);

    double delta_sum_1 = fabs(solns[0][0] - state.theta1) + fabs(solns[0][1] - state.theta2);
    double delta_sum_2 = fabs(solns[1][0] - state.theta1) + fabs(solns[1][1] - state.theta2);

    double* best;
    if (delta_sum_1 < delta_sum_2) {
        best = solns[0];
    } else {
        best = solns[1];
    }

    double delta1 = CalculateQuickestPath(state.theta1, best[0]);
    double delta2 = CalculateQuickestPath(state.theta2, best[1]);

    double mdelta1 = CalculateMotorDelta(delta1);
    double mdelta2 = CalculateMotorDelta(delta2);

    // Move the motors
    mdelta1 = mdelta1;  // Replace with motor functions, get the actual number of completed steps or degrees
    mdelta2 = mdelta2; // Replace with motor functions, get the actual number of completed steps or degrees

    #ifdef DEBUG
      PrintAnglesInDegrees(mdelta1, mdelta2);
    #endif

    state.theta1 += mdelta1;
    state.theta2 += mdelta2;

    double x_new, y_new;
    CalculateCartesianCoords(state.theta1, state.theta2, &x_new, &y_new);
    state.x = x_new;
    state.y = y_new;
}

/**
 * @brief Increment the robots current position.
 * 
 * @param rel_x x increment.
 * @param rel_y y increment.
 */
void MoveBy(double rel_x, double rel_y) {
    double new_x = state.x + rel_x;
    double new_y = state.y + rel_y;

    MoveTo(new_x, new_y);
}