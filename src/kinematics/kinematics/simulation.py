import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import csv
from forword_k import forward

# ================= CONSTANTS =================
sqrt3 = np.sqrt(3)
pi = np.pi
sin30 = 0.5
tan30 = 1 / sqrt3
tan60 = sqrt3

# ================= GEOMETRY =================
f = 0.439075 / 2     # base triangle side (m)
e = 0.200918 / 2     # end effector triangle side (m)
rf = 0.100           # bicep length (m)
re = 0.200           # forearm length (m)

# Direction vectors for each arm (matches forward()'s internal convention)
D1 = np.array([0.0, -1.0, 0.0])
D2 = np.array([sqrt3/2, 0.5, 0.0])
D3 = np.array([-sqrt3/2, 0.5, 0.0])
DIRS = [D1, D2, D3]

# ================= FIGURE =================
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.6, 0.1)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Delta Robot Simulator")

# ================= BASE TRIANGLE =================
L = f * sqrt3 / 2
R = f / (sqrt3 * 2)
A = np.array([-f/2, -R, 0])
B = np.array([ f/2, -R, 0])
C = np.array([ 0,  L-R, 0])

base = np.array([A, B, C, A])
ax.plot(base[:,0], base[:,1], base[:,2], 'r', lw=2)

# Real physical pivot points = base triangle edge midpoints (radius R, on the triangle)
base_pts = np.array([(A+B)/2, (B+C)/2, (C+A)/2])
PIVOTS = base_pts  # same as the edge midpoints, at radius R along DIRS

# ================= DRAW OBJECTS =================
upper = [ax.plot([], [], [], 'k', lw=3)[0] for _ in range(3)]
lower = [ax.plot([], [], [], 'b', lw=2)[0] for _ in range(3)]
ee_triangle, = ax.plot([], [], [], 'g', lw=2)
ee_point = ax.scatter([], [], [], c='r', s=40)

printed = False  # print once

# ================= EE TRAJECTORY =================
ee_path_x = []
ee_path_y = []
ee_path_z = []
ee_path_line, = ax.plot([], [], [], 'm--', lw=2)  # trajectory line

# ================= ANIMATION =================
def update(frame, thetaa):
    global printed

    # The client node negates theta before sending to the controller,
    # so the CSV stores -theta relative to inverse()/forward()'s convention.
    theta = -thetaa[frame]           # radians, in inverse()/forward()'s convention
    theta_deg = np.degrees(theta)    # forward() internally expects degrees

    ee = forward(*theta_deg)
    if ee is None:
        return

    ee = np.array(ee)

    # store trajectory AFTER ee is valid
    ee_path_x.append(ee[0])
    ee_path_y.append(ee[1])
    ee_path_z.append(ee[2])

    ee_path_line.set_data(ee_path_x, ee_path_y)
    ee_path_line.set_3d_properties(ee_path_z)

    # ---- End-effector triangle (visual scaling trick) ----
    scale = e / f
    center = (A + B + C) / 3

    Aee = ee + scale*(A - center)
    Bee = ee + scale*(B - center)
    Cee = ee + scale*(C - center)

    ee_triangle.set_data(
        [Aee[0], Bee[0], Cee[0], Aee[0]],
        [Aee[1], Bee[1], Cee[1], Aee[1]]
    )
    ee_triangle.set_3d_properties(
        [Aee[2], Bee[2], Cee[2], Aee[2]]
    )

    ee_pts = [
        (Aee + Bee) / 2,
        (Bee + Cee) / 2,
        (Cee + Aee) / 2
    ]

    # ---- Arms: elbow uses real pivot (on the base triangle) + forward()'s cos/sin convention ----
    for i in range(3):
        Pi = PIVOTS[i]
        Di = DIRS[i]
        th = theta[i]

        elbow = Pi + rf*math.cos(th)*Di + np.array([0.0, 0.0, -rf*math.sin(th)])

        upper[i].set_data([Pi[0], elbow[0]], [Pi[1], elbow[1]])
        upper[i].set_3d_properties([Pi[2], elbow[2]])

        P = ee_pts[i]
        lower[i].set_data([elbow[0], P[0]], [elbow[1], P[1]])
        lower[i].set_3d_properties([elbow[2], P[2]])

    ee_point._offsets3d = ([ee[0]], [ee[1]], [ee[2]])

    if not printed:
        print("FK End Effector (first frame):", ee)
        printed = True


csv_file = "delta_joint_log.csv"

joint_traj = []

with open(csv_file, newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        joint_traj.append([
            float(row["joint_1"]),
            float(row["joint_2"]),
            float(row["joint_3"])
        ])

joint_traj = np.array(joint_traj)
num_frames = len(joint_traj)
print("Frames:", num_frames)

ani = FuncAnimation(fig, update, frames=num_frames, interval=30, fargs=(joint_traj,))
plt.show()