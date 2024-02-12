
import math

DEBUG = False
LIGHT_DEBUG = False
VISUALIZER = False

# Robot Parameters
LINK_1 = 85     # mm
LINK_2 = 190    # mm


DEGREES_PER_STEP = math.radians(1.8)
M1_GEAR_REDUCTION = 2
M2_GEAR_REDUCTION = 2
RESOLUTION = DEGREES_PER_STEP / M1_GEAR_REDUCTION
THETA1_MAX = math.radians(160)
THETA1_MIN = math.radians(-160)
THETA2_MAX = math.radians(100)
THETA2_MIN = math.radians(-100)


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

    # Keep angles within [-180, 180]
    for i, soln in enumerate(solns):
        for j, coord in enumerate(soln):
            if coord > math.pi:
                solns[i][j] = coord - 2*math.pi
            elif coord < -math.pi:
                solns[i][j] = coord + 2*math.pi

    if LIGHT_DEBUG:
        print(f"Sol 1: ({math.degrees(solns[0][0])}, {math.degrees(solns[0][1])})")
        print(f"Sol 2: ({math.degrees(solns[1][0])}, {math.degrees(solns[1][1])})")

    return solns

def calculate_cartesian_coords(theta1, theta2):
    x = math.cos(theta1)*LINK_1 + math.cos(theta1 + theta2)*LINK_2
    y = math.sin(theta1)*LINK_1 + math.sin(theta1 + theta2)*LINK_2

    return x, y

# Returns the quickest path between two angles which does not move through the restricted range
def calculate_quickest_valid_path(cur_theta, targ_theta, link):
    THETA_MAX = THETA2_MAX
    THETA_MIN = THETA2_MIN
    if link == 1:
        THETA_MAX = THETA1_MAX
        THETA_MIN = THETA1_MIN
    
    delta = targ_theta - cur_theta
    # Normalize the delta
    if delta > math.pi:
        delta = delta - 2*math.pi
    elif delta < -math.pi:
        delta = delta + 2*math.pi
    # Prevent the robot from moving through restricted angle
    if (cur_theta + delta) > THETA_MAX:
        return delta - 2*math.pi
    elif (cur_theta + delta) < THETA_MIN:
        return delta + 2*math.pi 
    return delta

def calculate_motor_delta(delta):
    return round(delta/RESOLUTION) * RESOLUTION

# Rudimentary state machine
robot_state = {
    "homed": False,
    "inmotion": False,
    "theta1": THETA1_MAX,
    "theta2": THETA2_MAX,
}
robot_state["x"], robot_state["y"] = calculate_cartesian_coords(THETA1_MAX, THETA2_MAX)

def is_valid(soln):
    # Check if normalized angles are within the specified range in radians
    if soln[0] < THETA1_MIN or soln[0] > THETA1_MAX:
        return False
    elif soln[1] < THETA2_MIN or soln[1] > THETA2_MAX:
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
 
    # Check to see if solutions are valid
    soln1_valid = is_valid(sol1)
    soln2_valid = is_valid(sol2)

    # If both are valid, take the shortest path
    if soln1_valid and soln2_valid:
        # This is a very rudimentary way to determine the best path, will need improvement
        delta_sum_1 = abs(sol1[0] - cur_theta1) + abs(sol1[1] - cur_theta2)
        delta_sum_2 = abs(sol2[0] - cur_theta1) + abs(sol2[1] - cur_theta2)
        # Choose the best solution
        if delta_sum_1 < delta_sum_2:
            best = sol1
        else:
            best = sol2
    # If only one solution is valid, take that one
    elif soln1_valid:
        best = sol1
    elif soln2_valid:
        best = sol2
    # Need to determine behavior if neither solutions is valid
    else:
        # Do error stuff
        # Probably just blink the red status LED and ignore the command
        print("Invalid")
        return [robot_state["theta1"], robot_state["theta2"]]
    
    # Calculate the fastest path that does not move through the restricted angle range
    delta1 = calculate_quickest_valid_path(cur_theta1, best[0], 1)
    delta2 = calculate_quickest_valid_path(cur_theta2, best[1], 2)

    # Convert to something the motor can actually do
    mdelta1 = calculate_motor_delta(delta1)
    mdelta2 = calculate_motor_delta(delta2)

    # Update the state machine
    robot_state["theta1"] = cur_theta1 + mdelta1
    robot_state["theta2"] = cur_theta2 + mdelta2    
    robot_state["x"], robot_state["y"] = calculate_cartesian_coords(robot_state["theta1"], robot_state["theta2"])
    
    if VISUALIZER:
        return best
    return mdelta1, mdelta2

# For making relative position requests
def rel_request(rel_x, rel_y):
    cur_x = robot_state["x"]
    cur_y = robot_state["y"]

    new_x = cur_x + rel_x
    new_y = cur_y + rel_y

    return abs_request(new_x, new_y)


if not VISUALIZER:
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])
    print(abs_request(150, 150))
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])
    print(abs_request(160, 150))
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])
    print(abs_request(160, 160))
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])
    print(abs_request(150, 150))
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])
    print(abs_request(-115, -160))
    # print(robot_state["theta1"], robot_state["theta2"], robot_state["x"], robot_state["y"])



