import numpy as np
import matplotlib.pyplot as plt

from kinematics.forword_k import forward

theta_range = np.linspace(-30, 96.6, 35)  # degrees, joint limits
workspace = []

for t1 in theta_range:
    for t2 in theta_range:
        for t3 in theta_range:
            p = forward(t1, t2, t3)
            if p is not None:
                workspace.append(p)

workspace = np.array(workspace)
print("Workspace points:", workspace.shape[0])

if workspace.shape[0] == 0:
    raise RuntimeError("No valid workspace points found — check joint limits / geometry constants.")

# =====================================================
# PLOT WORKSPACE (MATPLOTLIB 3D)
# =====================================================
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection="3d")

ax.scatter(
    workspace[:, 0],
    workspace[:, 1],
    workspace[:, 2],
    s=1,
    c=workspace[:, 2],
    cmap="viridis"
)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")
ax.set_title("Delta Robot Workspace")

# extremes
idx_x_max, idx_x_min = np.argmax(workspace[:, 0]), np.argmin(workspace[:, 0])
idx_y_max, idx_y_min = np.argmax(workspace[:, 1]), np.argmin(workspace[:, 1])
idx_z_max, idx_z_min = np.argmax(workspace[:, 2]), np.argmin(workspace[:, 2])

print("X max:", workspace[idx_x_max])
print("X min:", workspace[idx_x_min])
print("Y max:", workspace[idx_y_max])
print("Y min:", workspace[idx_y_min])
print("Z max:", workspace[idx_z_max])
print("Z min:", workspace[idx_z_min])

ax.set_box_aspect([1, 1, 1])
plt.show()