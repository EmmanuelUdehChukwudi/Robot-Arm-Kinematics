import numpy as np
from math import atan2, acos, sqrt, cos, sin, radians, degrees
import sys
import serial
import time
from forward_kinematics import get_homogeneous_transform

# Robot arm dimensions
a1 = 14.0
a2 = 10.5
a3 = 19.0

# Target coordinates
x = 15.25
y = 20.1
z = 12.0

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
if theta2 < 0:
    theta2 = alpha + beta
phi = acos((a2**2 + a3**2 - r**2) / (2 * a2 * a3))
theta3 = phi - np.pi / 2

# Convert angles to degrees
theta1 = int(np.rad2deg(theta1))
theta2 = int(np.rad2deg(theta2))
theta3 = int(np.rad2deg(theta3))
theta4 = 0  # Initial guess for theta4

# Serial communication with Arduino
try:
    ser = serial.Serial('COM3', 115200, timeout=1)  # Change 'COM3' to your Arduino's port
    time.sleep(2)  # Wait for the connection to be established

    # Function to get the current positions from the Arduino
    def get_current_positions(serial_connection):
        serial_connection.write(b'?')
        while True:
            line = serial_connection.readline().decode().strip()
            if line:
                return list(map(int, line.split(',')))

    # Function to send incremental angles to Arduino
    def send_incremental_angles(serial_connection, command, start_angle, end_angle, steps=10):
        step_size = (end_angle - start_angle) / steps
        for i in range(steps + 1):
            angle = int(start_angle + i * step_size)
            message = f"{command}{angle},"
            serial_connection.write(message.encode())
            time.sleep()

    # Get current positions
    current_positions = get_current_positions(ser)
    current_waist, current_shoulder, current_elbow, current_wrist, current_wrist_twist, current_tool_tip = current_positions

    # Convert current angles to radians
    current_waist = radians(current_waist)
    current_shoulder = radians(current_shoulder)
    current_elbow = radians(current_elbow)
    current_wrist = radians(current_wrist)

    # Update the DH parameters with the current angles
    DH = np.array([
        [current_waist, np.pi / 2, 0.0, a1],
        [current_shoulder, np.pi, a2, 0.0],
        [current_elbow, np.pi / 2, 0.0, 0.0],
        [current_wrist, 0.0, 0.0, a3]
    ], dtype=np.float32)

    # Send incremental angles for smooth motion
    send_incremental_angles(ser, 'b', current_waist, radians(theta1))
    send_incremental_angles(ser, 's', current_shoulder, radians(theta2))
    send_incremental_angles(ser, 'e', current_elbow, radians(theta3))

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    ser.close()

