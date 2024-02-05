/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>

// Robot Parameters
#define LINK_1 85.0     // mm
#define LINK_2 190.0    // mm

#define DEGREES_PER_STEP (M_PI / 100.0) // Converted 1.8 degrees to radians
#define M1_GEAR_REDUCTION 2
#define M2_GEAR_REDUCTION 2
#define RESOLUTION (DEGREES_PER_STEP / M1_GEAR_REDUCTION)

#define HOMED_THETA1 120 / 180 * M_PI
#define HOMED_THETA2 M_PI

struct stateMachine {
  bool homed;
  bool inmotion;
  double theta1;
  double theta2;
  double x;
  double y;
};

extern struct stateMachine state;

void InitializeStateMachine(void);
void MoveTo(double x, double y);
void MoveBy(double rel_x, double rel_y);
void PrintAnglesInDegrees(double theta1, double theta2);
void PrintCaresianCoords(double x, double y);

#endif
