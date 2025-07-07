import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pathlib  
import subprocess, shlex
import os
import time
import numpy as np
from matplotlib.animation import FuncAnimation
# --------------------------------------------------------------------------------------------------------------------------------
# ------------------------------        AUTOREADER FOR POSITION & INVERSE JACOBIAN DATA    --------------- 07/04/25 -------------
# --------------------------------------------------------------------------------------------------------------------------------

WORKSPACE   = pathlib.Path(__file__).resolve().parents[3] #3rd parent up is just the crazyfly_ws where cluster_data is --> ~/crazyfly_ws
WORKSPACE2 = pathlib.Path(__file__).resolve().parents[3]
#  print(f"this is the workspace {WORKSPACE}")    
DATA_DIR    = WORKSPACE / "cluster_data"# <— save_data_to_csv() writes here
INV_DATA_DIR = WORKSPACE2 / "I_Joc_values"                       
PATTERN     = "cluster_data_*.csv"                             # matches all cluster logs
FILE_PATTERN = "cluster_dot_*.csv"

csv_files   = sorted(DATA_DIR.glob(PATTERN))                #finds the file through glob and sorted
I_Joc_files = sorted(INV_DATA_DIR.glob(FILE_PATTERN))

# ERROR STATEMENT
if not csv_files:
    raise FileNotFoundError(f"No files matching {PATTERN} in {DATA_DIR}")
if not I_Joc_files:
    raise FileNotFoundError(f"No file matching {FILE_PATTERN} in {INV_DATA_DIR}")

file_path   = csv_files[-1]            # newest because the timestamp sorts lexicographically
I_joc_path = I_Joc_files[-1]
print(f"[flightplots] LATEST FLIGHT: {file_path}")
print(f"[I_joc] LATEST INPUTS {I_joc_path}")

# open file to see data
# subprocess.run(shlex.split(f"code -r {file_path}"))

# --------------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------------




# Filepath to the CSV file
# file_path = "/home/rsl/crazyfly_ws/cluster_data/cluster_data_20250701_154916.csv"
# # Read the CSV file into a DataFrame
data = pd.read_csv(file_path)

col1 = 'Des_CF2_Y'
col2 = 'Des_CF2_Y'
col3 = 'Cur_CF2_Y'

# Convert the 'Timestamp' column to datetime
data['Timestamp'] = pd.to_datetime(data['Timestamp'])
'''
# Ensure the data columns are numeric and handle any potential issues with missing or invalid data
data['Cur_CF1_Y'] = pd.to_numeric(data['Cur_CF1_Y'], errors='coerce')
data['Des_CF1_Y'] = pd.to_numeric(data['Des_CF1_Y'], errors='coerce')

# Drop rows with NaN values in the relevant columns
data = data.dropna(subset=['Timestamp', 'Cur_CF1_Y', 'Des_CF1_Y'])

# Convert the relevant columns to numpy arrays for compatibility with matplotlib
timestamps = data['Timestamp'].to_numpy()
line1 = data[col1].to_numpy()
line2 = data[col2].to_numpy()
line3 = data[col3].to_numpy()

# Plot the time series for columns 2 (Cur_CF1_X) and 8 (Cur_Cluster_X)
plt.figure(figsize=(12, 6))
plt.plot(timestamps, line1, label=col1, color='blue')
plt.plot(timestamps, line2, label=col2, color='orange')
plt.plot(timestamps, line3, label=col3, color='green')

# Add labels, title, and legend
plt.xlabel('Timestamp')
plt.ylabel('Values')
plt.title(f'Time Series Plot of {col1} and {col2} and {col3}')
plt.legend()
plt.grid()

# Show the plot
plt.tight_layout()
plt.show()
 '''
# Convert the relevant columns to numpy arrays for compatibility with matplotlib
cur_cf1_positions = data[['Cur_CF1_X', 'Cur_CF1_Z', 'Cur_CF1_Y']].to_numpy()
cur_cf2_positions = data[['Cur_CF2_X', 'Cur_CF2_Z', 'Cur_CF2_Y']].to_numpy()
cur_cluster_positions = data[['Cur_Cluster_X', 'Cur_Cluster_Z', 'Cur_Cluster_Y']].to_numpy()
des_cluster_positions = data[['Des_Cluster_X', 'Des_Cluster_Z', 'Des_Cluster_Y']].to_numpy()
des_cf1_positions = data[['Des_CF1_X', 'Des_CF1_Z', 'Des_CF1_Y']].to_numpy()
des_cf2_positions = data[['Des_CF2_X', 'Des_CF2_Z', 'Des_CF2_Y']].to_numpy()

# Plot 3D position data
fig = plt.figure("3D Positions")
ax = fig.add_subplot(111, projection="3d")

# Plot each dataset
ax.scatter(cur_cf1_positions[:, 0], cur_cf1_positions[:, 1], cur_cf1_positions[:, 2], label='Cur_CF1', c='r')
ax.scatter(cur_cf2_positions[:, 0], cur_cf2_positions[:, 1], cur_cf2_positions[:, 2], label='Cur_CF2', c='g')
ax.scatter(cur_cluster_positions[:, 0], cur_cluster_positions[:, 1], cur_cluster_positions[:, 2], label='Cur_Cluster', c='b')
ax.scatter(des_cluster_positions[:, 0], des_cluster_positions[:, 1], des_cluster_positions[:, 2], label='Des_Cluster', c='y')
ax.scatter(des_cf1_positions[:, 0], des_cf1_positions[:, 1], des_cf1_positions[:, 2], label='Des_CF1', c='m')
ax.scatter(des_cf2_positions[:, 0], des_cf2_positions[:, 1], des_cf2_positions[:, 2], label='Des_CF2', c='c')

# Normalize the axes to start at 0
ax.set_xlim(left=0)
ax.set_ylim(bottom=0)
ax.set_zlim(bottom=0)

# Label axes
ax.set_xlabel('X Position')
ax.set_ylabel('Z Position')
ax.set_zlabel('Y Position')
ax.legend()
plt.title('3D Position Data')




# --------------------------------- INVERSE JACOBIAN PLOTS -------------------------------------
df = pd.read_csv(I_joc_path)   # same path as logger
df["rel_time"] = df["time_s"] - df["time_s"].iloc[0]  #iloc is pandas method for selected rows and columns. "integer-location"
t = df["rel_time"].to_numpy()

fig_IJ= plt.figure("Inverse Jacobian Commands to Drones")
ack = fig_IJ.add_subplot(111)


ack.plot(t, df["x1dot"].to_numpy(), label="x1_dot")
ack.plot(t, df["x2dot"].to_numpy(), label="x2_dot")
ack.plot(t, df["y1dot"].to_numpy(), label="y1_dot")
ack.plot(t, df["y2dot"].to_numpy(), label="y2_dot")
ack.plot(t, df["z1dot"].to_numpy(), label="z1_dot")
ack.plot(t, df["z2dot"].to_numpy(), label="z2_dot")
ack.set_xlabel("Time [s]"); ack.set_ylabel("Commanded velocity [m/s]")
ack.legend(); ack.grid(True); fig_IJ.tight_layout(); 
plt.show()



# --------------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------          ANIMATIONS             ---------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------------


# # storing and setting up values for the first positional graphplt.show

# max_frames = min(len(data), len(df))  #sync the index for animation

# fig_anim_pos = plt.figure("3D Position Animations")
# an_pos = fig_anim_pos.add_subplot(111, projection ="3d")

# (line_cf1_cur, )= an_pos.plot([], [], [], c ="r", label = "Cf1_cur") # This uses tuple-unpacking (the '(line_cf1_cur, )' )
# (line_cf2_cur, )= an_pos.plot([], [], [], c ="g", label = "Cf2_cur")
# (line_clus_cur, )= an_pos.plot([], [], [], c ="b", label = "Clus_cur")
# (line_clus_des, ) = an_pos.plot([], [], [], c ="y", label = "Clus_des")
# (line_cf1_des, )= an_pos.plot([], [], [], c ="m", label = "Cf1_des")
# (line_cf2_des, )= an_pos.plot([], [], [], c ="c", label = "Cf2_des")

# an_pos.set_xlabel("X"); an_pos.set_ylabel("Y"); an_pos.set_zlabel("Z")
# an_pos.legend()
# an_pos.set_xlim(cur_cf1_positions[:,0].min(), cur_cf1_positions[:,0].max())
# an_pos.set_ylim(cur_cf1_positions[:,2].min(), cur_cf1_positions[:,2].max())
# an_pos.set_zlim(cur_cf1_positions[:,1].min(), cur_cf1_positions[:,1].max())



# # setting up the velocity graph
# fig_anim_IJ = plt.figure("Inverse Jacobian Commands Animated")
# an_IJ = fig_anim_IJ.add_subplot(111)


# anim_x1dot, = an_IJ.plot([],[], label = "x1_dot")
# anim_x2dot, = an_IJ.plot([],[], label = "x2_dot")
# anim_y1dot, = an_IJ.plot([],[], label = "y1_dot")
# anim_y2dot, = an_IJ.plot([],[], label = "y2_dot")
# anim_z1dot, = an_IJ.plot([],[], label = "z1_dot")
# anim_z2dot, = an_IJ.plot([],[], label = "z2_dot")

# an_IJ.set_xlim(0, t.max())
# an_IJ.set_ylim(df.drop(columns="rel_time").min().min(), df.drop(columns="rel_time").max().max())
# an_IJ.set_xlabel("Time (s)")
# an_IJ.set_ylabel("command velocities (m/s)")
# an_IJ.legend(); an_IJ.grid(True)

# def update(frame):
#     line_cf1_cur.set_data(cur_cf1_positions[:frame,0], cur_cf1_positions[:frame,2])
#     line_cf1_cur.set_3d_properties(cur_cf1_positions[:frame,1])

#     line_cf1_des.set_data(cur_cf2_positions[])






#     anim_x1dot.set_data((t[:frame], df["x1dot"].iloc[:frame]))
#     anim_x2dot.set_data((t[:frame], df["x2dot"].iloc[:frame]))
#     anim_y1dot.set_data((t[:frame], df["y1dot"].iloc[:frame]))
#     anim_y2dot.set_data((t[:frame], df["y2dot"].iloc[:frame]))
#     anim_z1dot.set_data((t[:frame], df["z1dot"].iloc[:frame]))
#     anim_z2dot.set_data((t[:frame], df["z2dot"].iloc[:frame]))
    

#     return(line_cf1_cur)



# anim_pos = FuncAnimation(fig_anim_pos,update,frames = max_frames, interval = 50, blit = False)
# ani_vel = FuncAnimation(fig_anim_IJ, update, frames=max_frames, interval=50, blit=False)
# plt.show()

fig = plt.figure("3D Positions Over Time")
ax = fig.add_subplot(111, projection="3d")

all_positions = np.vstack([
    cur_cf1_positions,
    cur_cf2_positions,
    cur_cluster_positions,
    des_cluster_positions,
    des_cf1_positions,
    des_cf2_positions,
])


padding = 0.05
x_min, x_max = all_positions[:, 0].min(), all_positions[:, 0].max()
y_min, y_max = all_positions[:, 1].min(), all_positions[:, 1].max()
z_min, z_max = all_positions[:, 2].min(), all_positions[:, 2].max()

x_range = x_max - x_min
y_range = y_max - y_min
z_range = z_max - z_min

ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
#ax.set_ylim(z_min - padding * z_range, z_max + padding * z_range)  # Z is on Y-axis
#ax.set_zlim(y_max + padding * y_range, y_min - padding * y_range)
ax.set_ylim(0,2)
ax.set_zlim(0,1)

ax.set_xlabel('X Position')
ax.set_ylabel('Z Position')
ax.set_zlabel('Y Position')

# trail scatters
scatters = {
    'Cur_CF1': ax.scatter([], [], [], color='r', label='Cur_CF1'),
    'Cur_CF2': ax.scatter([], [], [], color='g', label='Cur_CF2'),
    'Cur_Cluster': ax.scatter([], [], [], color='b', label='Cur_Cluster'),
    'Des_Cluster': ax.scatter([], [], [], color='y', label='Des_Cluster'),
    'Des_CF1': ax.scatter([], [], [], color='m', label='Des_CF1'),
    'Des_CF2': ax.scatter([], [], [], color='c', label='Des_CF2'),
}
ax.legend()
plt.title('3D Position Data')

datasets = {
    'Cur_CF1': cur_cf1_positions,
    'Cur_CF2': cur_cf2_positions,
    'Cur_Cluster': cur_cluster_positions,
    'Des_Cluster': des_cluster_positions,
    'Des_CF1': des_cf1_positions,
    'Des_CF2': des_cf2_positions,
}

def update(frame):
    for key, scatter in scatters.items():
        pos = datasets[key]
        if frame < len(pos):
            xs = pos[:frame+1, 0]
            ys = pos[:frame+1, 1]
            zs = pos[:frame+1, 2]
            scatter._offsets3d = (xs, ys, zs)
    return scatters.values()

num_frames = min(len(p) for p in datasets.values())
ani = FuncAnimation(fig, update, frames=num_frames, interval=100, blit=False)
#plt.show()

#Animation for 2D Graph

# Load and preprocess data
#df = pd.read_csv(I_joc_path)
# df["rel_time"] = df["time_s"] - df["time_s"].iloc[0]
# t = df["rel_time"].to_numpy()

# Extract each signal as a NumPy array
# x1dot = df["x1dot"].to_numpy()
# x2dot = df["x2dot"].to_numpy()
# y1dot = df["y1dot"].to_numpy()
# y2dot = df["y2dot"].to_numpy()
# z1dot = df["z1dot"].to_numpy()
# z2dot = df["z2dot"].to_numpy()

step = 43

# Downsampled time and signals
t2 = df["rel_time"].to_numpy()[::step]
x1dot = df["x1dot"].to_numpy()[::step]
x2dot = df["x2dot"].to_numpy()[::step]
y1dot = df["y1dot"].to_numpy()[::step]
y2dot = df["y2dot"].to_numpy()[::step]
z1dot = df["z1dot"].to_numpy()[::step]
z2dot = df["z2dot"].to_numpy()[::step]

# Create animated figure
fig_IJ = plt.figure("Inverse Jacobian Commands to Drones")
ack = fig_IJ.add_subplot(111)

ack.set_xlabel("Time [s]")
ack.set_ylabel("Commanded velocity [m/s]")
ack.grid(True)
ack.set_xlim(t[0], t2[-1])
ack.set_ylim(df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().min() - 0.1,
              df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().max() + 0.1)

# Initialize lines
(line_x1,) = ack.plot([], [], label="x1_dot")
(line_x2,) = ack.plot([], [], label="x2_dot")
(line_y1,) = ack.plot([], [], label="y1_dot")
(line_y2,) = ack.plot([], [], label="y2_dot")
(line_z1,) = ack.plot([], [], label="z1_dot")
(line_z2,) = ack.plot([], [], label="z2_dot")

ack.legend()
fig_IJ.tight_layout()

# Update function for animation
def update_1(frame):
    line_x1.set_data(t2[:frame], x1dot[:frame])
    line_x2.set_data(t2[:frame], x2dot[:frame])
    line_y1.set_data(t2[:frame], y1dot[:frame])
    line_y2.set_data(t2[:frame], y2dot[:frame])
    line_z1.set_data(t2[:frame], z1dot[:frame])
    line_z2.set_data(t2[:frame], z2dot[:frame])
    return line_x1, line_x2, line_y1, line_y2, line_z1, line_z2

# Create animation
ani_2d = FuncAnimation(fig_IJ, update_1, frames=len(t2), interval=100, blit=True)
plt.show()
