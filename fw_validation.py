import numpy as np
import sympy
from sympy import Matrix, pi, cos, sin
import matplotlib.pyplot as plt

# Define symbolic variables
theta1, theta2, theta3, l1, l2, l3 = sympy.symbols("theta1, theta2, theta3, l1, l2, l3")

# Transformation matrix from DH parameters
def transformation_matrix_from_dh_param(dh_row):
    theta, alpha, a, d = dh_row
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Define DH tables for both legs
left_leg_dh_table = Matrix([
    [theta1, -pi/2, 0, 0],
    [0, 0, 0, l1],
    [theta2 + pi/4, 0, l2, 0],
    [theta3 - pi/2, 0, l3, 0]
])

right_leg_dh_table = Matrix([
    [theta1, pi/2, 0, 0],
    [0, 0, 0, l1],
    [theta2 - pi/4, 0, l2, 0],
    [theta3 + pi/2, 0, l3, 0]
])

# Calculate transformation matrices
def calculate_transformation_matrix(dh_table):
    return [transformation_matrix_from_dh_param(dh_table[i, :]) for i in range(dh_table.rows)]

def forward_kinematics(transformation_matrix):
    matrix = Matrix.eye(4)
    fk_steps = []  # Store intermediate transformations
    for transform in transformation_matrix:
        matrix = matrix * transform
        fk_steps.append(matrix)
    return matrix, fk_steps

# Generate FK for each leg
fk_left, fk_left_steps = forward_kinematics(calculate_transformation_matrix(left_leg_dh_table))
fk_right, fk_right_steps = forward_kinematics(calculate_transformation_matrix(right_leg_dh_table))

# Define joint configurations and validate FK
configurations = [
    {"leg": "Left", "theta1": 0, "theta2": -pi/4, "theta3": pi/2, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Left", "theta1": 0, "theta2": -pi/6, "theta3": pi/3, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Right", "theta1": 0, "theta2": pi/4, "theta3": -pi/2, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Right", "theta1": 0, "theta2": pi/6, "theta3": -pi/3, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
]

for config in configurations:
    leg = config["leg"]
    theta1_val, theta2_val, theta3_val = config["theta1"], config["theta2"], config["theta3"]
    l1_val, l2_val, l3_val = config["l1"], config["l2"], config["l3"]
   
    fk_matrix = fk_left if leg == "Left" else fk_right
    fk_result = fk_matrix.subs({
        theta1: theta1_val,
        theta2: theta2_val,
        theta3: theta3_val,
        l1: l1_val,
        l2: l2_val,
        l3: l3_val
    }).evalf()
   
    print(f"{leg} Leg Configuration: theta1={theta1_val}, theta2={theta2_val}, theta3={theta3_val}")
    print("Transformation Matrix:")
    sympy.pprint(fk_result)
    print("-" * 50)

def plot_robot(fk_steps, config, ax, title):
    # Extract link lengths and joint angles
    l1_val, l2_val, l3_val = config["l1"], config["l2"], config["l3"]
    theta1_val, theta2_val, theta3_val = config["theta1"], config["theta2"], config["theta3"]
   
    # Calculate joint positions
    points = [(0, 0, 0)]  # Base point
    for step in fk_steps:
        pos = step[:3, 3].evalf(subs={
            theta1: theta1_val, theta2: theta2_val, theta3: theta3_val,
            l1: l1_val, l2: l2_val, l3: l3_val
        })
        pos_numeric = [float(p) for p in pos]  # Convert to numerical values
        points.append(pos_numeric)
   
    points = np.array(points)
   
    # Plot configuration
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', label='Leg Structure')
    ax.legend()
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

# Create a figure with subplots for each configuration
fig = plt.figure(figsize=(16, 12))

for i, config in enumerate(configurations):
    ax = fig.add_subplot(2, 2, i+1, projection='3d')
    fk_steps = fk_left_steps if config["leg"] == "Left" else fk_right_steps
    plot_robot(fk_steps, config, ax, title=f"{config['leg']} Leg\nθ1={config['theta1']} θ2={config['theta2']} θ3={config['theta3']}")

plt.tight_layout()
plt.show()


