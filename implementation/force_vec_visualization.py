import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from PIL.ImageOps import scale
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import three_Tether_Dynamic_Model_eqns
import params
import os

eqns = three_Tether_Dynamic_Model_eqns.three_teth_model()
p = params.three_teth_Parameters()

# test data
name = "nocontroll_xoscillations"
df = pd.read_csv("Trimmed_Dynamic_Testing_Data/TRIMMED_tether_data_20250416_nocontroll_xoscillations.csv")

# Define the folder path
folder = 'plots/' + name
# Create the folder if it doesn't exist
os.makedirs(folder, exist_ok=True)


times = df['ResponseTime'].to_numpy() / 1000
positions = np.column_stack((df['XApex'], df['YApex'], df['ZApex']))
f = np.column_stack((df['Loadcell1Force'], df['Loadcell2Force'], df['Loadcell3Force']))

# Add tether anchor points (static red crosses)
teth_anchor = np.array(p.teth_anchor)

# Define reference vector (assuming this is constant for all points)
# Replace this with your actual reference vector calculation if it varies by point
f_ref = np.array([0.0, 0.0, p.mass])  # Example reference vector - replace with your actual reference

num_rows = positions.shape[0]
vectors = np.zeros((num_rows, 3))
ref_vectors = np.zeros((num_rows, 3))  # Reference vectors

for i in range(num_rows):
    # Extract single row of position and force data
    pos_i = positions[i]
    f_i = f[i]

    # Calculate tether error vectors for this row
    _, _, teth1_vec, teth2_vec, teth3_vec = eqns.calculate_tether_error(pos_i, f_i)

    # Store reference vector (if it's constant, this is redundant but makes the code more general)
    ref_vectors[i] = f_ref/np.linalg.norm(f_ref)*2

    # Sum the vectors and store in result array
    vectors[i] = teth1_vec + teth2_vec + teth3_vec

    vectors[i] = vectors[i]/ np.linalg.norm(f_ref)*2



# Set up the figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Calculate axis limits with some padding
pad = 1.5
x_range = teth_anchor[:, 0].max() - teth_anchor[:, 0].min()
y_range = teth_anchor[:, 1].max() - teth_anchor[:, 1].min()
z_range = positions[:, 2].max() - positions[:, 2].min()
max_range = max(x_range, y_range, z_range) / 2.0

mid_x = (teth_anchor[:, 0].max() + teth_anchor[:, 0].min()) * 0.5
mid_y = (teth_anchor[:, 1].max() + teth_anchor[:, 1].min()) * 0.5
mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5

ax.set_xlim(mid_x - max_range * pad, mid_x + max_range * pad)
ax.set_ylim(mid_y - max_range * pad, mid_y + max_range * pad)

# FIX: Reverse the z-axis limits to fix the inverted z-axis
z_min = 0
z_max = -max_range * pad
ax.set_zlim(z_min, z_max)  # Inverted z-axis


# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Point and Vector Animation')

# Add a grid
ax.grid(True)


anchor_plot = ax.scatter(
    teth_anchor[:, 0], teth_anchor[:, 1], teth_anchor[:, 2],
    color='k', marker='x', s=60, label='Tether Anchors'
)

# Create objects to animate
point = ax.plot([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 'ro', markersize=8)[0]
trajectory = ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b--', alpha=0.3)[0]

# Create initial quiver plots for both vectors
quiver = ax.quiver(
    positions[0, 0], positions[0, 1], positions[0, 2],
    vectors[0, 0], vectors[0, 1], vectors[0, 2],
    color='g', length=1.0, normalize=True, arrow_length_ratio=0.1
)

# Reference vector in red
ref_quiver = ax.quiver(
    positions[0, 0], positions[0, 1], positions[0, 2],
    ref_vectors[0, 0], ref_vectors[0, 1], ref_vectors[0, 2],
    color='r', length=1.0, normalize=True, arrow_length_ratio=0.1
)

# Text annotation for time
time_text = ax.text2D(0.05, 0.95, "Time: 0.0", transform=ax.transAxes)


# Function to interpolate between data points
def interpolate(pos1, pos2, vec1, vec2, ref1, ref2, t):
    pos = pos1 + t * (pos2 - pos1)
    vec = vec1 + t * (vec2 - vec1)
    ref = ref1 + t * (ref2 - ref1)
    return pos, vec, ref


# Animation update function
def update(frame):
    global quiver, ref_quiver  # Important: use the global quiver objects

    # Get the frame index and fractional part for interpolation
    frame_idx = int(frame)
    fraction = frame - frame_idx

    # Handle the last frame special case
    if frame_idx >= len(times) - 1:
        frame_idx = len(times) - 2
        fraction = 1.0

    # Interpolate position and vectors
    pos, vec, ref_vec = interpolate(
        positions[frame_idx],
        positions[frame_idx + 1],
        vectors[frame_idx],
        vectors[frame_idx + 1],
        ref_vectors[frame_idx],
        ref_vectors[frame_idx + 1],
        fraction
    )

    # Update point position
    point.set_data([pos[0]], [pos[1]])
    point.set_3d_properties([pos[2]])

    # Remove previous quivers
    quiver.remove()
    ref_quiver.remove()

    # Create new quivers
    quiver = ax.quiver(
        pos[0], pos[1], pos[2],
        vec[0], vec[1], vec[2],
        color='g', length=1.0, normalize=False, arrow_length_ratio=0.1
    )

    ref_quiver = ax.quiver(
        pos[0], pos[1], pos[2],
        ref_vec[0], ref_vec[1], ref_vec[2],
        color='r', length=1.0, normalize=False, arrow_length_ratio=0.1
    )

    # Update time text
    t = times[frame_idx] + fraction * (times[frame_idx + 1] - times[frame_idx])
    time_text.set_text(f"Time: {t:.1f}")

    return point, quiver, ref_quiver, time_text


# Match animation duration to actual time span
fps = 25
total_duration = times[-1] - times[0]  # in seconds
num_frames = int(total_duration * fps)
interval = 1000 / fps  # in milliseconds

# Generate frame indices with interpolation
frames = np.linspace(0, len(times) - 1.001, num_frames)

ani = FuncAnimation(
    fig, update, frames=frames, interval=interval,
    blit=False, repeat=False
)

# Add a legend
from matplotlib.lines import Line2D

legend_elements = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=10, label='Apex'),
    Line2D([0], [0], color='g', lw=2, label='Measure Force'),
    Line2D([0], [0], color='r', lw=2, label='Reference Force'),
    Line2D([0], [0], color='b', linestyle='--', lw=2, label='Trajectory'),
    Line2D([0], [0], marker='x', color='k', markersize=10, label='Tether Anchors')
]
ax.legend(handles=legend_elements, loc='upper right')

# Adjust the initial view angle for better 3D perspective
# ax.view_init(elev=20, azim=135)  # You can change these values to adjust the view
ax.view_init(elev=5, azim=90)  # You can change these values to adjust the view

plt.tight_layout()


# Save the animation using FFMpegWriter
from matplotlib.animation import FFMpegWriter
matplotlib.rcParams['animation.ffmpeg_path'] = r'C:\Users\Peter\OneDrive\Desktop\ffmpeg\ffmpeg-2024-11-18-git-970d57988d-full_build\bin\ffmpeg.exe'
writer = FFMpegWriter(fps=25, metadata=dict(artist='Peter'), bitrate=1800)
ani.save('plots/' + name + '/tether_animation' + name + '.mp4', writer=writer)

print("Animation saved")


plt.show()