import numpy as np
from math import pi, cos, sin, atan2, sqrt, radians, degrees

# Transformations
def rotx(ang):
    return np.array([[1, 0, 0],
                     [0, cos(ang), -sin(ang)],
                     [0, sin(ang), cos(ang)]])

def roty(ang):
    return np.array([[cos(ang), 0, sin(ang)],
                     [0, 1, 0],
                     [-sin(ang), 0, cos(ang)]])

def rotz(ang):
    return np.array([[cos(ang), -sin(ang), 0],
                     [sin(ang), cos(ang), 0],
                     [0, 0, 1]])

def rotxyz(x_ang, y_ang, z_ang):
    return np.matmul(np.matmul(rotx(x_ang), roty(y_ang)), rotz(z_ang))

def homog_transxyz(x, y, z):
    return np.block([[np.eye(3), np.array([[x], [y], [z]])],
                     [np.zeros((1, 3)), np.array([1])]])

def homog_rotxyz(x_ang, y_ang, z_ang):
    return np.block([[rotxyz(x_ang, y_ang, z_ang), np.zeros((3, 1))],
                     [np.zeros((1, 3)), np.array([1])]])

def ht_inverse(ht):
    rot_inv = ht[:3, :3].transpose()
    trans_inv = -np.matmul(rot_inv, ht[:3, 3])
    return np.block([[rot_inv, trans_inv.reshape(3, 1)],
                     [np.zeros((1, 3)), np.array([1])]])

# Forward Kinematics
def t_0_to_1(theta1, l1):
    return np.block([[rotz(theta1), np.array([[-l1 * cos(theta1)], [-l1 * sin(theta1)], [0]])],
                     [np.zeros((1, 3)), np.array([1])]])

def t_1_to_2():
    return np.array([[0, 0, -1, 0],
                     [-1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]])

def t_2_to_3(theta2, l2):
    return np.block([[rotz(theta2), np.array([[l2 * cos(theta2)], [l2 * sin(theta2)], [0]])],
                     [np.zeros((1, 3)), np.array([1])]])

def t_3_to_4(theta3, l3):
    return np.block([[rotz(theta3), np.array([[l3 * cos(theta3)], [l3 * sin(theta3)], [0]])],
                     [np.zeros((1, 3)), np.array([1])]])

def t_0_to_4(theta1, theta2, theta3, l1, l2, l3):
    return np.matmul(np.matmul(np.matmul(t_0_to_1(theta1, l1), t_1_to_2()),
                               t_2_to_3(theta2, l2)), t_3_to_4(theta3, l3))

# Inverse Kinematics
def ikine(x4, y4, z4, l1, l2, l3, legs12=True):
    D = (x4**2 + y4**2 - l1**2 + z4**2 - l2**2 - l3**2) / (2 * l2 * l3)
    if legs12:
        q3 = atan2(sqrt(1 - D**2), D)
    else:
        q3 = atan2(-sqrt(1 - D**2), D)

    q2 = -atan2(z4, sqrt(x4**2 + y4**2 - l1**2)) - atan2(l3 * sin(q3), l2 + l3 * cos(q3))
    q1 = atan2(-y4, x4) - atan2(sqrt(x4**2 + y4**2 - l1**2), -l1)

    return -q1, -q2, -q3

# Robot Leg Parameters
hip_length = 0.0255
upper_leg_length = 0.10921
lower_leg_length = 0.135

# Helper functions for angle conversion
def deg_to_rad(angle_list):
    return [radians(angle) for angle in angle_list]

def rad_to_deg(angle_list):
    return [degrees(angle) for angle in angle_list]

# Validate FK and IK
# Validate FK and IK with legs12 parameter based on the leg configuration
def validate_kinematics(joint_angles, legs12):
    joint_angles_rad = deg_to_rad(joint_angles)
    
    # Forward Kinematics
    fk_matrix = t_0_to_4(joint_angles_rad[0], joint_angles_rad[1], joint_angles_rad[2],
                         hip_length, upper_leg_length, lower_leg_length)
    foot_position = fk_matrix[:3, 3]
    
    # Inverse Kinematics
    ik_angles = ikine(foot_position[0], foot_position[1], foot_position[2],
                      hip_length, upper_leg_length, lower_leg_length, legs12)
    ik_angles_deg = rad_to_deg(ik_angles)
    
    return {
        "Original Angles (deg)": joint_angles,
        "Foot Position (m)": foot_position.tolist(),
        "IK Angles (deg)": ik_angles_deg,
    }

# Test joint angles with corresponding legs12 values
joint_angles_list = [
    ([0.0, 45, 90.0], False),   # Leg 1
    ([11.5735, -33.0804, 100.5692], False), # Leg 2
    ([11.5735, 33.0804, -100.569], True),  # Leg 3
    ([7.5883, -28.7493, 29.7695], False),  # Leg 4
]

# Run validation
results = []
for i, (angles, legs12) in enumerate(joint_angles_list):
    print(f"Validating Leg {i+1}")
    result = validate_kinematics(angles, legs12)
    results.append(result)

# Print results
for i, result in enumerate(results):
    print(f"\nLeg {i+1} Validation Results:")
    for key, value in result.items():
        print(f"{key}: {value}")
