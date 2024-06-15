import numpy as np

#Discretize the robot arm configuration space
joint_ranges = {
    'base': (0,180,10),
    'shoulder': (0,180,10),
    'elbow': (0,180,10),
    'wrist': (0,180,10)
}
def generate_joint_positions(min_angle, max_angle, step_size):
    return np.arange(min_angle, max_angle + step_size, step_size)

base = generate_joint_positions(*joint_ranges['base'])
shoulder = generate_joint_positions(*joint_ranges['shoulder'])
elbow = generate_joint_positions(*joint_ranges['elbow'])
wrist = generate_joint_positions(*joint_ranges['wrist'])

robot_configurartion_space = []

for b in base:
    for s in shoulder:
        for e in elbow:
            for w in wrist:
                robot_configurartion_space.append((b,s,e,w))
