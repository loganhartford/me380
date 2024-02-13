#include "controls.h"
#include "motor_hal.h"
#include "main.h"

// #define DEBUG // Enables serial print statements

void CalculateJointAngle(double x, double y, double solns[2][2]);
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y);
double CalculateQuickestValidPath(double cur_theta, double targ_theta, bool link1);
double CalculateMotorDelta(double delta);
bool IsValid(double *soln);

void dummy(void){
    // goToAngle(M_PI, 2*M_PI, 0);
}

/**
 * @brief Initializes the robot state
 * 
 */
    void InitializeStateMachine(void)
{
    state.homed = 1;                                                      // Homed yet
    state.inmotion = 0;                                                   // Not in motion
    state.grasping = 0;                                                   // Not grasping
    state.theta1 = THETA1_MAX;                                            // Link 1 in homed position
    state.theta2 = THETA2_MAX;                                            // Link 2 in home position
    CalculateCartesianCoords(THETA1_MAX, THETA2_MAX, &state.x, &state.y); // Determine homed x, an y position
}

/**
 * @brief Converts cartesian coordinates to joint angles of the robot.
 *
 * @param x coordinate.
 * @param y coordinate.
 * @param solns An array which will contain the set(2) of possible angles.
 */
void CalculateJointAngle(double x, double y, double solns[2][2])
{
    // Calculate length of end effector vector
    double R = sqrt(x * x + y * y);

    // Angle of vector using atan2 to handle quadrants
    double THETA = atan2(y, x);

    // Handle the limits of acos
    double acosarg = (R * R - LINK_1 * LINK_1 - LINK_2 * LINK_2) / (-2 * LINK_1 * LINK_2);
    double beta;
    if (acosarg < -1.0)
    {
        beta = M_PI;
    }
    else if (acosarg > 1.0)
    {
        beta = 0.0;
    }
    else
    {
        beta = acos(acosarg);
    }

    double alpha;
    double break_r = sqrt(LINK_2 * LINK_2 - LINK_1 * LINK_1);
    if (R > break_r)
    {
        alpha = asin((LINK_2 * sin(beta)) / R);
    }
    else if (R > 0.0)
    {
        alpha = M_PI / 2 + (M_PI / 2 - asin((LINK_2 * sin(beta)) / R));
    }
    else
    {
        alpha = 0.0;
    }

    // Assembly both possible solutions [theta1, theta2]
    solns[0][0] = THETA - alpha;
    solns[0][1] = M_PI - beta;
    solns[1][0] = THETA + alpha;
    solns[1][1] = beta - M_PI;

    // Keep angles within [-180, 180]
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (solns[i][j] > M_PI)
            {
                solns[i][j] -= 2 * M_PI;
            }
            else if (solns[i][j] < -M_PI)
            {
                solns[i][j] += 2 * M_PI;
            }
        }
    }

#ifdef DEBUG
    PrintAnglesInDegrees(solns[0][0], solns[0][1]);
    PrintAnglesInDegrees(solns[1][0], solns[1][1]);
#endif
}

void PrintState()
{
    printf("Robot State:\n\r");
    printf("Homed: %s\n\r", state.homed ? "Yes" : "No");
    printf("In Motion: %s\n\r", state.inmotion ? "Yes" : "No");
    printf("Grasping: %s\n\r", state.grasping ? "Yes" : "No");
    PrintAnglesInDegrees(state.theta1, state.theta2);
    PrintCaresianCoords(state.x, state.y);
    printf("\n\r");
}

/**
 * @brief Prints two radian angles in degrees to the serial monitor.
 *
 * @param theta1 Radian angle 1.
 * @param theta2 Radian angle 2.
 */
void PrintAnglesInDegrees(double theta1, double theta2)
{
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
void PrintCaresianCoords(double x, double y)
{
    int int_part = (int)x;
    int decimal_part = abs((int)((x - int_part) * 100)); // 2 decimal places
    int int_part2 = (int)y;
    int decimal_part2 = abs((int)((y - int_part2) * 100)); // 2 decimal places

    printf("(%d.%d, %d.%d)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Converts cartesion coordinates to link angles of the robot.
 *
 * @param theta1 angle of link 1.
 * @param theta2 angle of link 2/
 * @param x position
 * @param y position
 */
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y)
{
    *x = cos(theta1) * LINK_1 + cos(theta1 + theta2) * LINK_2;
    *y = sin(theta1) * LINK_1 + sin(theta1 + theta2) * LINK_2;

#ifdef DEBUG
    PrintCaresianCoords(*x, *y);
#endif
}

/**
 * @brief Caclculates the smallest absolute able between two angles.
 *
 * @param cur_theta current angle.
 * @param targ_theta desired angle.
 * @return double - the smallest delta to get from the current to the desired angle.
 */
double CalculateQuickestValidPath(double cur_theta, double targ_theta, bool link1)
{
    double THETA_MAX = THETA2_MAX;
    double THETA_MIN = THETA2_MIN;
    if (link1)
    {
        THETA_MAX = THETA1_MAX;
        THETA_MIN = THETA1_MIN;
    }

    double delta = targ_theta - cur_theta;
    // Normalize the delta
    if (delta > M_PI)
    {
        return delta - 2 * M_PI;
    }
    else if (delta < -M_PI)
    {
        return delta + 2 * M_PI;
    }
    // Prevent the robot from moving through the restricted angle
    if ((cur_theta + delta) > THETA_MAX)
    {
        return delta - 2 * M_PI;
    }
    else if ((cur_theta - delta) < THETA_MIN)
    {
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
double CalculateMotorDelta(double delta)
{
    return round(delta / RESOLUTION) * RESOLUTION;
}

bool IsValid(double *soln)
{
    // Check if normalized angles are within the specified range in radians
    if ((soln[0] < THETA1_MIN) || (soln[0] > THETA1_MAX))
    {
        return false;
    }
    else if ((soln[1] < THETA2_MIN) || (soln[1] > THETA2_MAX))
    {
        return false;
    }
    return true;
}

/**
 * @brief Move the robot to the x and y coordinates.
 *
 * @param x coordniate.
 * @param y coordinate.
 */
void MoveTo(double x, double y)
{
    double solns[2][2];
    CalculateJointAngle(x, y, solns);

    // Check to see if solutions are valid
    bool soln1_valid = IsValid(solns[0]);
    bool soln2_valid = IsValid(solns[1]);

    double *best;
    // If both solutions are valid, take the quicker one
    if (soln1_valid && soln2_valid)
    {
        double delta_sum_1 = fabs(solns[0][0] - state.theta1) + fabs(solns[0][1] - state.theta2);
        double delta_sum_2 = fabs(solns[1][0] - state.theta1) + fabs(solns[1][1] - state.theta2);

        if (delta_sum_1 < delta_sum_2)
        {
            best = solns[0];
        }
        else
        {
            best = solns[1];
        }
        /*
        Note:
            - This is not the best way to do this
            - When we caculate the delta_sum, we are not taking into account the inability to move through the restricted angle range.
            - This could result in taking the less ideal solution
        */
    }
    // If only one solution is valid, take that one
    else if (soln1_valid)
    {
        best = solns[0];
    }
    else if (soln2_valid)
    {
        best = solns[1];
    }
    // If neither solution is valid, error
    else
    {
        // Blink status LED
        printf("Invalid Request\n\r");
        return;
    }

    double delta1 = CalculateQuickestValidPath(state.theta1, best[0], true);
    double delta2 = CalculateQuickestValidPath(state.theta2, best[1], false);

    double mdelta1 = CalculateMotorDelta(delta1);
    double mdelta2 = CalculateMotorDelta(delta2);

    // Move the motors

    double realdelta1;
    double realdelta2;
    double realdeltaz;
    goToAngle(mdelta1, mdelta2, 0.0, &realdelta1, &realdelta2, &realdeltaz);

#ifdef DEBUG
    PrintAnglesInDegrees(mdelta1, mdelta2);
#endif
    PrintAnglesInDegrees(mdelta1, mdelta2);

    // Update the state machine
    state.theta1 += mdelta1;
    state.theta2 += mdelta2;
    CalculateCartesianCoords(state.theta1, state.theta2, &state.x, &state.y);
}

/**
 * @brief Increment the robots current position.
 *
 * @param rel_x x increment.
 * @param rel_y y increment.
 */
void MoveBy(double rel_x, double rel_y)
{
    double new_x = state.x + rel_x;
    double new_y = state.y + rel_y;

    MoveTo(new_x, new_y);
}