import numpy as np
import matplotlib.pyplot as plt
import math

"""
This works but takes a long time to compute and then the graph breaks if you try and move it lol.
"""

def calculate_position(theta1, theta2, l1, l2):
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return x, y

def main():
    l1 = 85  # Length of the first link in mm
    l2 = 190 # Length of the second link in mm
    step_angle = 1.8 # Motor step angle in degrees
    gear_reduction = 2 # Gear reduction ratio

    # Adjust step angle for gear reduction
    effective_step_angle = step_angle / gear_reduction

    # Convert step angle to radians
    step_radians = math.radians(effective_step_angle)

    # Prepare plot
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Robot Arm Reachable Positions')
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')

    # Calculate reachable positions
    for theta1 in np.arange(0, 2*math.pi, step_radians):
        for theta2 in np.arange(-math.pi, math.pi, step_radians):
            x, y = calculate_position(theta1, theta2, l1, l2)
            ax.plot(x, y, 'ro', markersize=1)

    plt.show()

if __name__ == "__main__":
    main()
