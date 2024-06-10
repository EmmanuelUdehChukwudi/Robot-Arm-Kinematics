import numpy as np
from math import cos, sin

# dummy link lenghts
a1 = 14.0
a2 = 10.5
a3 = 19.0

#set intial joint angles to zero
T1 = np.deg2rad(-90)
T2 = np.deg2rad(17)
T3 = np.deg2rad(0)
T4 = np.deg2rad(0)

#Denavit-Hartenburg table
DH = np.array([[T1, np.pi/2, 0.0, a1],
               [T2, np.pi, a2, 0.0],
               [T3, np.pi/2, 0.0, 0.0],
               [T4, 0.0, 0.0, a3]], dtype=np.float32)


def get_homogeneous_transform(theta, alpha, r, d):
    transform = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float32)
    
    transform = np.round(transform, 3)
    return transform


def return_end_effector_position():
    #get transforms between frames
    H0_1 = get_homogeneous_transform(theta=DH[0][0], alpha=DH[0][1], r=DH[0][2], d=DH[0][3])
    H1_2 = get_homogeneous_transform(theta=DH[1][0], alpha=DH[1][1], r=DH[1][2], d=DH[1][3])
    H2_3 = get_homogeneous_transform(theta=DH[2][0], alpha=DH[2][1], r=DH[2][2], d=DH[2][3])
    H3_4 = get_homogeneous_transform(theta=DH[3][0], alpha=DH[3][1], r=DH[3][2], d=DH[3][3])

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)
    end_effector_postion = H0_4[:3,-1]
    end_effector_oreintation = H0_4[:3,:3]
    x = end_effector_postion[0]
    y = end_effector_postion[1]
    z = end_effector_postion[2]
    return (x,y,z)

print(return_end_effector_position())