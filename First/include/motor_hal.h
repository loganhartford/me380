// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "limit_switch_hal.h"
#include "stm32f4xx_hal_tim.h"

#define STEPS_PER_REV 6400.0
#define MIN_RPM 5.0
#define MAX_RPM 20.0
#define Z_STEPS_PER_REV 400

#define CW 0
#define CCW 1

typedef struct
{
    const char *name;
    GPIO_TypeDef *stepPort;   // Port of the motor
    uint16_t stepPin;         // Pin for stepping
    GPIO_TypeDef *dirPort;    // Port for the direction
    uint16_t dirPin;          // Pin to set direction
    bool dir;                 // Motor direction
    int reduction;            // Gear reduction of the motor
    double thetaMax;          // Positive limit switch position
    double thetaMin;          // Negative limit switch position
    uint32_t stepsToComplete; // Number of steps the motor has left to complete
    uint32_t stepsToSpeedUp;  // How many steps the motor has to ramp up speed
    uint32_t stepsToSlowDown; // How many steps the motor has to ramp down speed
    double slope;             // The slope betweent the min and target speed
    double currentRPM;        // The motors current rpm
    bool isMoving;            // Is the motor moving?
    LimitSwitch limitSwitch;  // Limit switch associated with the motor

} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motorz;

void Motors_Init(void);
double MoveByAngle(Motor *motor, double angle, double speedRPM);
double MoveByDist(Motor *motor, double dist, double speedRPM);
void HomeMotors(void);
void StopMotors(void);

#endif