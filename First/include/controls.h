// Define to prevent recursive inclusion
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>

#define LINK_1 85.0     // mm
#define LINK_2 190.0    // mm

#define DEGREES_PER_STEP (M_PI / 100.0) // Converted 1.8 degrees to radians
#define M1_GEAR_REDUCTION 2
#define M2_GEAR_REDUCTION 2
#define RESOLUTION (DEGREES_PER_STEP / M1_GEAR_REDUCTION)

#define HOMED_THETA1 0
#define HOMED_THETA2 0


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

#endif
