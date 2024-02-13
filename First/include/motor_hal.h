// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "stm32f4xx_hal_tim.h"

#define STEPS_PER_REV 6400.0
#define DEGREES_PER_STEP (360.0 / STEPS_PER_REV)
#define RADS_PER_STEP (2 * M_PI / STEPS_PER_REV)
#define MOTOR1_RED 1
#define MOTOR2_RED 2
#define Z_STEPS_PER_REV 400
#define Z_RADS_PER_STEP (2 * M_PI / Z_STEPS_PER_REV)

#define CW 0
#define CCW 1

typedef struct
{
    GPIO_TypeDef *stepPort; // Port of the motor
    uint16_t stepPin;       // Pin for stepping
    GPIO_TypeDef *dirPort;
    uint16_t directionPin; // Pin to set direction
    double currentAngle;   // Current angle of the motor
    double targetAngle;    // Target angle for the motor to move to
    double rpm;            // Rotation speed in RPM
    int direction;         // Direction of movement
    bool moveDone;         // Flag to indicate if the move is done
    double radsPerStep;    // Radians per step of the motor
    int reduction;         // Gear reduction of the motor
} Motor;

extern Motor motor_x;
extern Motor motor_y;
extern Motor motor_z;

void Motors_Init(void);
void MoveByAngle(double theta1, double theta2, double thetaz, double *realtheta1, double *realtheta2, double *realthetaz);

#endif