import numpy as np
import serial
import time
from forward_kinematics import get_homogeneous_transform
import sys

a1 = 14.0
a2 = 10.5
a3 = 19.0


def is_reachable(x, y, z):
    max_pos = a1 + a2 + a3
    return np.sqrt(x**2 + y**2 + z**2) <= max_pos


def inverse_kinematics(x, y, z):
    r1 = np.sqrt(x**2 + y**2)
    theta1 = np.arctan2(y, x)
    r2 = z - a1
    r = np.sqrt(r1**2 + r2**2)
    alpha = np.arctan2(r2, r1)
    beta = np.arccos((a2**2 + r**2 - a3**2) / (2 * a2 * r))
    theta2 = alpha - beta
    phi = np.arccos((a2**2 + a3**2 - r**2) / (2 * a2 * a3))
    theta3 = phi - np.pi / 2
    DH = np.array([[theta1, np.pi/2, 0.0, a1],
                   [theta2, np.pi, a2, 0.0],
                   [theta3, np.pi/2, 0.0, 0.0],
                   ], dtype=np.float32)
    R0_4 = [[0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]]
    R0_1 = get_homogeneous_transform(theta=DH[0][0], alpha=DH[0][1], r=DH[0][2], d=DH[0][3])[:3, :3]
    R1_2 = get_homogeneous_transform(theta=DH[1][0], alpha=DH[1][1], r=DH[1][2], d=DH[1][3])[:3, :3]
    R2_3 = get_homogeneous_transform(theta=DH[2][0], alpha=DH[2][1], r=DH[2][2], d=DH[2][3])[:3, :3]

    R0_3 = np.dot(np.dot(R0_1, R1_2), R2_3)
    R0_3_inv = np.transpose(R0_3)
    R3_4 = np.dot(R0_3_inv, R0_4)

    theta4 = np.arctan2(R3_4[1][0], R3_4[0][0])

    return [int(np.rad2deg(theta1)), int(np.rad2deg(theta2)), int(np.rad2deg(theta3)), int(np.rad2deg(theta4))]


def get_current_positions(serial_connection):
    serial_connection.write(b'?')
    while True:
        line = serial_connection.readline().decode().strip()
        if line:
            positions = list(map(int, line.split(',')))
            print("Received positions:", positions)
            if len(positions) == 5:
                return positions
            else:
                print("Invalid number of positions received:", len(positions))


def send_angles(serial_connection, angles):
    message = f"b{angles[0]},s{angles[1]},e{angles[2]},w{angles[3]},"
    serial_connection.write(message.encode())
    print(f"Sent to Arduino: {message}")


def send_incremental_angles(serial_connection, current_positions, target_positions, step_size=1):
    steps = int(max(abs(target_positions[i] - current_positions[i]) // step_size for i in range(len(current_positions))))
    print("Steps:", steps)
    if steps == 0:
        print("Robot already at starting position")
    for step in range(steps + 1):
        incremental_angles = [
            int(current_positions[i] + (target_positions[i] - current_positions[i]) * step / steps)
            for i in range(len(current_positions))
        ]
        send_angles(serial_connection, incremental_angles)
        time.sleep(0.1)


def modify_angles(angles):
    theta1, theta2, theta3, theta4 = angles
    if theta1 < 0:
        theta1 = 180 + theta1
    if theta2 < 0:
        theta2 = 180 + theta2
    if theta3 < 0:
        theta3 = 180 + theta3
    if theta4 < 0:
        theta4 = 180 + theta4

    return [theta1, theta2, theta3, theta4]


def main():
    try:
        port = input("Enter the serial port: ")
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)

        while True:
            # Get current positions from Arduino
            current_positions = get_current_positions(ser)
            current_positions = current_positions[:4]  # I removed the gripper
            print(f"Current positions: {current_positions}")

            # Get target positions from inverse kinematics
            x = float(input("Enter x coordinates: "))
            y = float(input("Enter y coordinates: "))
            z = float(input("Enter z coordinates: "))
            if not is_reachable(x, y, z):
                print("Target position is out of reach.")
                continue
            target_positions = inverse_kinematics(x, y, z)
            old = target_positions
            print(f"Target positions: {target_positions}")
            for i in target_positions:
                if i < 0:
                    print("Cannot send command to robot because one or more of the joints is negative. Finding alternate solutions.....")
                    new = modify_angles(old)
                    send_incremental_angles(ser, current_positions, new)
                    break
                    # ser.close()
                    # sys.exit()
            else:
                send_incremental_angles(ser, current_positions, target_positions)

            choice = input("Do you want to enter new coordinates? (yes/no): ").lower()
            if choice != 'yes':
                break

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
