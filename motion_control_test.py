import numpy as np
from math import atan2, acos, sqrt, radians, degrees
import sys
import time
from forward_kinematics import get_homogeneous_transform

# Robot arm dimensions
a1 = 14.0
a2 = 10.5
a3 = 19.0

# Target coordinates
x = 2.76
y = 13.0
z = 36.0

# Check if target is reachable
max_pos = a1 + a2 + a3
if sqrt(x**2 + y**2 + z**2) > max_pos:
    print("Coordinates Out of range of the end effector")
    sys.exit()

# Inverse kinematics calculations
r1 = sqrt(x**2 + y**2)
theta1 = atan2(y, x)
r2 = z - a1
r = sqrt(r1**2 + r2**2)
alpha = atan2(r2, r1)
beta = acos((a2**2 + r**2 - a3**2) / (2 * a2 * r))
theta2 = alpha - beta

phi = acos((a2**2 + a3**2 - r**2) / (2 * a2 * a3))
theta3 = phi - np.pi / 2

# Convert angles to degrees
theta1 = int(np.rad2deg(theta1))
theta2 = int(np.rad2deg(theta2))
theta3 = int(np.rad2deg(theta3))
theta4 = 0  # Initial guess for theta4

print(f"""
      theta1 = {theta1}
      theta2 = {theta2}
      theta3 = {theta3}
      theta4 = {theta4}
      """)
# Ensure all angles are positive
if theta1 < 0:
    theta1 += 360
if theta2 < 0:
    theta2 += 360
if theta3 < 0:
    theta3 += 360

# Simulate current positions from Arduino
current_positions = [30, 45, 60, 0, 0]  # Example current positions for waist, shoulder, elbow, wrist, tool_tip
current_waist, current_shoulder, current_elbow, current_wrist, current_tool_tip = current_positions

# Function to simulate sending angles to Arduino
def send_angles(waist, shoulder, elbow, wrist, tool_tip):
    message = f"b{waist},s{shoulder},e{elbow},w{wrist},g{tool_tip},"
    print(f"Sending to Arduino: {message}")

# Function to send incremental angles to Arduino
def send_incremental_angles(current_positions, target_positions):
    current_waist, current_shoulder, current_elbow, current_wrist, current_tool_tip = current_positions
    target_waist, target_shoulder, target_elbow, target_wrist, target_tool_tip = target_positions

    while (current_waist != target_waist or
           current_shoulder != target_shoulder or
           current_elbow != target_elbow or
           current_wrist != target_wrist):
        
        if current_waist < target_waist:
            current_waist += 1
        elif current_waist > target_waist:
            current_waist -= 1
        
        if current_shoulder < target_shoulder:
            current_shoulder += 1
        elif current_shoulder > target_shoulder:
            current_shoulder -= 1
        
        if current_elbow < target_elbow:
            current_elbow += 1
        elif current_elbow > target_elbow:
            current_elbow -= 1
        
        if current_wrist < target_wrist:
            current_wrist += 1
        elif current_wrist > target_wrist:
            current_wrist -= 1

        send_angles(current_waist, current_shoulder, current_elbow, current_wrist, current_tool_tip)
        time.sleep(0.1)  # Adjust this delay for smoother motion

# Target positions
target_positions = [theta1, theta2, theta3, theta4, current_tool_tip]

# Send incremental angles for smooth motion
send_incremental_angles(current_positions, target_positions)
