// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "limit_switch_hal.h"
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
    const char *name;
    GPIO_TypeDef *stepPort; // Port of the motor
    uint16_t stepPin;       // Pin for stepping
    GPIO_TypeDef *dirPort;
    uint16_t dirPin; // Pin to set direction
    bool dir;
    int reduction;   // Gear reduction of the motor
    double thetaMax; // Positive limit switch position
    double thetaMin; // Negative limit switch position
    uint32_t stepsToComplete;
    bool isMoving;
    LimitSwitch limitSwitch;

} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motorz;

void Motors_Init(void);
double MoveByAngle(Motor *motor, double angle, double speedRPM);
void HomeMotors(void);
void StopMotors(void);

#endif