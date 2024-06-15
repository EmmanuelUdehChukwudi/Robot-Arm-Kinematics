import numpy as np
from math import atan2,acos,sqrt
from  forward_kinematics import get_homogeneous_transform
import sys


a1 = 14.0
a2 = 10.5
a3 = 19.0

x = -16.82
y = -18.68
z = 19.74

max_pos = a1+a2+a3
if sqrt(x**2 + y**2 + z**2) > max_pos:
     print("Coordinates Out of range of the end effector")
     sys.exit()

r1 = sqrt(x**2 + y**2)
theta1 = atan2(y,x)
r2 = z - a1
r = sqrt(r1**2 + r2**2)
alpha = atan2(r2,r1)
beta = acos((a2**2 +r**2 - a3**2)/(2*a2*r))
theta2 = (alpha - beta)

phi = acos((a2**2 +a3**2 - r**2)/(2*a2*a3))

theta3 = phi - np.pi/2



DH = np.array([[theta1, np.pi/2, 0.0, a1],
               [theta2, np.pi, a2, 0.0],
               [theta3, np.pi/2, 0.0, 0.0],
               ], dtype=np.float32)
R0_4 = [[0,0,0],
        [0,0,0],
        [0,0,0]]
R0_1 = get_homogeneous_transform(theta=DH[0][0], alpha=DH[0][1], r=DH[0][2], d=DH[0][3])[:3,:3]
R1_2 = get_homogeneous_transform(theta=DH[1][0], alpha=DH[1][1], r=DH[1][2], d=DH[1][3])[:3,:3]
R2_3 = get_homogeneous_transform(theta=DH[2][0], alpha=DH[2][1], r=DH[2][2], d=DH[2][3])[:3,:3]


R0_3 = np.dot(np.dot(R0_1,R1_2),R2_3)
R0_3_inv = np.transpose(R0_3)
R3_4 = np.dot(R0_3_inv,R0_4)

theta4 = atan2(R3_4[1][0],R3_4[0][0])

theta1 = int(np.rad2deg(theta1))
theta2 = int(np.rad2deg(theta2))
theta3 = int(np.rad2deg(theta3))
theta4 = int(np.rad2deg(theta4))

print(f"""
     theta1 = {theta1}
     theta2 = {theta2}
     theta3 = {theta3}
     theta4 = {theta4}
      """)