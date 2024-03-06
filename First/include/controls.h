// Define to prevent recursive inclusion
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "motor_hal.h"
#include "main.h"

#define LINK_1 85.0  // mm
#define LINK_2 190.0 // mm
#define SAFETY_MARGIN 5.0 / 180.0 * M_PI

/**
 * @brief Robot state machine
 *
 */
struct stateMachine
{
    bool homed;    // Has the robot been homed
    bool grasping; // Is the robot grasping and object
    double theta1; // Link 1 angle
    double theta2; // Link 2 angle
    double x;      // X position of end effector
    double y;      // Y position of end effector
};

extern struct stateMachine state;

void InitializeStateMachine(void);
void MoveTo(double x, double y);
void MoveBy(double rel_x, double rel_y);
void PrintAnglesInDegrees(double theta1, double theta2);
void PrintCaresianCoords(double x, double y);
void PrintState(void);

void dummy(void);

#endif
