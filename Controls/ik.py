
import math

DEBUG = False

# Robot Parameters
LINK_1 = 85     # mm
LINK_2 = 190    # mm
HOMED_ANGLES = [math.radians(120), math.pi]
THETA1 = {}
THETA1["MAX"], THETA1["MIN"] = math.radians(120), -math.radians(120)
# RANGE_2

DEGREES_PER_STEP = 1.8
M1_GEAR_REDUCTION = 2
M2_GEAR_REDUCTION = 2

def calculate_joint_angles(x, y):
    # Calculate length of end effector vector
    R = math.sqrt(x**2 + y**2)
    if DEBUG: print(R)
    
    # Angle of vector
    if x == 0: x = 0.01
    THETA = math.atan(y/x)

    # Account for sign of tan
    if (x <= 0 and y >= 0) or (x <= 0 and y <= 0):
        THETA = math.pi + THETA
    elif x >= 0 and y <= 0:
        THETA = 2*math.pi + THETA
    
    if DEBUG: print("Theta: ", math.degrees(THETA))

    # Handle the limits of acos
    acosarg = (R**2 - LINK_1**2 - LINK_2**2)/(-2*LINK_1*LINK_2)
    if acosarg < -1.0:
        beta = math.pi
    elif acosarg > 1.0:
        beta = 0.0
    else:
        beta = math.acos(acosarg)
    if DEBUG: print("Beta: ", math.degrees(beta))
    
    break_r = math.sqrt(LINK_2**2 - LINK_1**2)
    if R > break_r:
         alpha = math.asin((LINK_2*math.sin(beta))/R)
         if DEBUG: print("Alpha :", math.degrees(alpha))
    elif R > 0.0:
        alpha = math.pi/2 + (math.pi/2 - math.asin((LINK_2*math.sin(beta))/R))
    else:
        alpha = 0.0

    # Assembly both possible solutions [theta1, theta2]
    sol1 = [THETA - alpha, math.pi - beta]
    sol2 = [THETA + alpha, beta - math.pi]
    solns = [sol1, sol2]

    # Keep angles within [-360, 360]
    for i, soln in enumerate(solns):
        for j, coord in enumerate(soln):
            if coord > 2*math.pi:
                solns[i][j] = coord - 2*math.pi
            elif coord < -2*math.pi:
                solns[i][j] = coord + 2*math.pi

    if DEBUG:
        print(f"Sol 1: ({math.degrees(solns[0][0])}, {math.degrees(solns[0][1])})")
        print(f"Sol 2: ({math.degrees(solns[1][0])}, {math.degrees(solns[1][1])})")

    return solns

def calculate_cartesian_coords(theta1, theta2):
    x = math.cos(theta1)*LINK_1 + math.cos(theta1 + theta2)*LINK_2
    y = math.sin(theta1)*LINK_1 + math.sin(theta1 + theta2)*LINK_2

    return x, y

# Rudimentary state machine
robot_state = {
    "theta1": HOMED_ANGLES[0],
    "theta2": HOMED_ANGLES[1],
}
robot_state["x"], robot_state["y"] = calculate_cartesian_coords(HOMED_ANGLES[0], HOMED_ANGLES[1])

# Check to see if the request motion will intersect the base
def is_soln_valid(soln, cur_theta1):
    theta1 = soln[0]
    # If the theta1 will be moving in the postive direction
    if theta1 > cur_theta1:
        # Check to see if the max is in between these angles
        if theta1 >= THETA1["MAX"] and cur_theta1 <= THETA1["MAX"]:
            return False
        return True
     # If the theta1 will be moving in the negative direction
    else:
        # Check to see if the min is in between these angles
        if theta1 <= THETA1["MIN"] and cur_theta1 >= THETA1["MIN"]:
            return False
        return True

# For making absolute position requests
def abs_request(x, y):
    angle_set = calculate_joint_angles(x , y)
    sol1 = angle_set[0]
    sol2 = angle_set[1]

    # Load in current pos
    cur_theta1 = robot_state["theta1"]
    cur_theta2 = robot_state["theta2"]

    # Check to see if either of the solutions are valid
    sol1_valid = is_soln_valid(sol1, cur_theta1)
    sol2_valid = is_soln_valid(sol2, cur_theta1)

    # If both solutions are valid, take the one that's closer
    if sol1_valid and sol2_valid:
        # Calculate the travel between cur pos and both potential pos
        delta_sum_1 = abs(sol1[0] - cur_theta1) + abs(sol1[1] - cur_theta2)
        delta_sum_2 = abs(sol2[0] - cur_theta1) + abs(sol2[1] - cur_theta2)

        # Choose the best solution
        if delta_sum_1 < delta_sum_2:
            best = sol1
        else:
            best = sol2
    # If only one solution is valid, take that one
    elif sol1_valid:
        best = sol1
    elif sol2_valid:
        best = sol2
    # If neither solutions is valid
    else:
        print("shit")

    robot_state["theta1"] = best[0]
    robot_state["theta2"] = best[1]

    robot_state["x"], robot_state["y"] = calculate_cartesian_coords(best[0], best[1])
    return best

# For making relative position requests
def rel_request(rel_x, rel_y):
    cur_x = robot_state["x"]
    cur_y = robot_state["y"]

    new_x = cur_x + rel_x
    new_y = cur_y + rel_y

    best = abs_request(new_x, new_y)
    
    return best




"""
* Set limits on x and y values
    * check if the abs command is valid and if not, replace with valid coords
    * check if the rel command is valid and if not replace with valid coords
* Test demo on laptop
"""
    



