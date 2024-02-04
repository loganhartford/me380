#include "main.h"

// #define IK_DEBUG
#define DEBUG

UART_HandleTypeDef UartHandle;

static void SystemClock_Config(void);
static void Error_Handler(void);
void SerialInit(void);
void initializeStateMachine(void);
void calculate_joint_angles(double x, double y, double solns[2][2]);
void print_angles_in_degrees(double theta1, double theta2);
void calculate_cartesian_coords(double theta1, double theta2, double *x, double *y);
double calculate_quickest_path(double cur_theta, double targ_theta);
double calculate_motor_delta(double delta);

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
#define HOMED_THETA1 calculate_motor_delta(120 * 180 / M_PI)
#define HOMED_THETA2 calculate_motor_delta(M_PI)

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
  SystemClock_Config();
  SerialInit();
  initializeStateMachine();
  
  double solns[2][2];
  while (1) {
    HAL_Delay(2000);
    printf("\n\r");
    
    // print_angles_in_degrees(delta1, delta2);
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

void SystemClock_Config(void)
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
    Error_Handler();
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
    Error_Handler();
  }

}

static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}

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
    Error_Handler();
  }
}

void calculate_joint_angles(double x, double y, double solns[2][2]) {
    // Calculate length of end effector vector
    double R = sqrt(x * x + y * y);

    // Angle of vector using atan2 to handle quadrants
    double THETA = atan2(y, x);

    // Handle the limits of acos
    double acosarg = (R * R - LINK_1 * LINK_1 - LINK_2 * LINK_2) / (-2 * LINK_1 * LINK_2);
    double beta;
    if (acosarg < -1.0) {
        printf("beta1");
        beta = M_PI;
    } else if (acosarg > 1.0) {
        beta = 0.0;
        printf("beta2");
    } else {
        beta = acos(acosarg);
        printf("beta3");
    }

    double alpha;
    double break_r = sqrt(LINK_2 * LINK_2 - LINK_1 * LINK_1);
    if (R > break_r) {
        printf("alpha1");
        alpha = asin((LINK_2 * sin(beta)) / R);
    } else if (R > 0.0) {
        printf("alpha2");
        alpha = M_PI / 2 + (M_PI / 2 - asin((LINK_2 * sin(beta)) / R));
    } else {
        printf("alpha3");
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
    print_angles_in_degrees(solns[0][0], solns[0][1]);
    print_angles_in_degrees(solns[1][0], solns[1][1]);
#endif
}

void print_angles_in_degrees(double theta1, double theta2) {
     theta1 = theta1 * (180.0 / M_PI);
     theta2 = theta2 * (180.0 / M_PI);

    int int_part = (int)theta1;
    int decimal_part = abs((int)((theta1 - int_part) * 100)); // 2 decimal places
    int int_part2 = (int)theta2;
    int decimal_part2 = abs((int)((theta2 - int_part2) * 100)); // 2 decimal places

    printf("(%d.%d degrees, %d.%d degrees)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

void initializeStateMachine(void) {
  
  state.homed = 0;
  state.inmotion = 0;
  state.theta1 = HOMED_THETA1;
  state.theta2 = HOMED_THETA2;
  calculate_cartesian_coords(HOMED_THETA1, HOMED_THETA2, &state.x, &state.y);
}

void calculate_cartesian_coords(double theta1, double theta2, double *x, double *y) {
    *x = cos(theta1) * LINK_1 + cos(theta1 + theta2) * LINK_2;
    *y = sin(theta1) * LINK_1 + sin(theta1 + theta2) * LINK_2;
}

// Function to determine the quickest path between two angles
double calculate_quickest_path(double cur_theta, double targ_theta) {
    double delta = targ_theta - cur_theta;
    if (delta > M_PI) {
        return delta - 2 * M_PI;
    } else if (delta < -M_PI) {
        return delta + 2 * M_PI;
    }
    return delta;
}

// Function to calculate the motor delta
double calculate_motor_delta(double delta) {
    return round(delta / RESOLUTION) * RESOLUTION;
}