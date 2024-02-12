// Define to prevent recursive inclusion
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#define LINK_1 85.0     // mm
#define LINK_2 190.0    // mm

#define DEGREES_PER_STEP (1.8 / 180.0 * M_PI) // Converted 1.8 degrees to radians
#define M1_GEAR_REDUCTION 2
#define M2_GEAR_REDUCTION 2
#define RESOLUTION (DEGREES_PER_STEP / M1_GEAR_REDUCTION)

#define THETA1_MIN (-160.0 / 180.0 * M_PI)
#define THETA1_MAX (160.0 / 180.0 * M_PI)
#define THETA2_MIN (-100.0 / 180.0 * M_PI)
#define THETA2_MAX (100.0 / 180.0 * M_PI)


/**
 * @brief Robot state machine
 * 
 */
struct stateMachine {
  bool homed;       // Has the robot been homed
  bool inmotion;    // Is the robot in motion
  bool grasping;    // Is the robot grasping and object
  double theta1;    // Link 1 angle
  double theta2;    // Link 2 angle
  double x;         // X position of end effector
  double y;         // Y position of end effector
};

extern struct stateMachine state;

void InitializeStateMachine(void);
void MoveTo(double x, double y);
void MoveBy(double rel_x, double rel_y);
void PrintAnglesInDegrees(double theta1, double theta2);
void PrintCaresianCoords(double x, double y);
void PrintState(void);

#endif
