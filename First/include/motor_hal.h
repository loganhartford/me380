// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "stm32f4xx_hal_tim.h"

#define STEPS_PER_REV 6400.0
// #define DEGREES_PER_STEP (360.0 / STEPS_PER_REV)
// #define RADS_PER_STEP (2 * M_PI / STEPS_PER_REV)
// #define MOTOR1_RED 1
// #define MOTOR2_RED 2
#define Z_STEPS_PER_REV 400
// #define Z_RADS_PER_STEP (2 * M_PI / Z_STEPS_PER_REV)

#define CW 0
#define CCW 1

typedef struct
{
    GPIO_TypeDef *stepPort; // Port of the motor
    uint16_t stepPin;       // Pin for stepping
    GPIO_TypeDef *dirPort;
    uint16_t dirPin; // Pin to set direction
    bool dir;
    double radsPerStep;  // Radians per step of the motor
    int reduction;       // Gear reduction of the motor
    double thetaMax;     // Positive limit switch position
    double thetaMin;     // Negative limit switch position
    bool limitTriggered; // Has/is a limit switch triggered?
    uint32_t stepsToComplete;

} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motorz;

void Motors_Init(void);
void MoveTheta1(double angle, double speedRPM);
void MoveTheta2(double angle, double speedRPM);
void HomeMotors(void);
void StopMotors(void);

#endif