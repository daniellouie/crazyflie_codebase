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
WORKSPACE3  = pathlib.Path(__file__).resolve().parents[3]

DATA_DIR    = WORKSPACE / "cluster_data"# <— save_data_to_csv() writes here
INV_DATA_DIR = WORKSPACE2 / "I_Joc_values"  
CF2_TUNING = WORKSPACE3 / "cf2_tuning"     

PATTERN     = "cluster_data_*.csv"                             # matches all cluster logs
FILE_PATTERN = "cluster_dot_*.csv"
FILE_PATTERN2 = "cf2_tuning_*.csv"

csv_files   = sorted(DATA_DIR.glob(PATTERN))                #finds the file through glob and sorted
I_Joc_files = sorted(INV_DATA_DIR.glob(FILE_PATTERN))
cf2_files = sorted(CF2_TUNING.glob(FILE_PATTERN2))

# ERROR STATEMENT
if not csv_files:
    raise FileNotFoundError(f"No files matching {PATTERN} in {DATA_DIR}")
if not I_Joc_files:
    raise FileNotFoundError(f"No file matching {FILE_PATTERN} in {INV_DATA_DIR}")
if not cf2_files:
    raise FileNotFoundError(f"No file matching {FILE_PATTERN2} in {CF2_TUNING}")

file_path   = csv_files[-1]            # newest because the timestamp sorts lexicographically
I_joc_path = I_Joc_files[-1]
cf2_path = cf2_files[-1]

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



# Convert the 'Timestamp' column to datetime
data['Timestamp'] = pd.to_datetime(data['Timestamp'])

# Ensure the data columns are numeric and handle any potential issues with missing or invalid data
data['Cur_CF1_Y'] = pd.to_numeric(data['Cur_CF1_Y'], errors='coerce')
data['Des_CF1_Y'] = pd.to_numeric(data['Des_CF1_Y'], errors='coerce')

# Drop rows with NaN values in the relevant columns
data = data.dropna(subset=['Timestamp', 'Cur_CF1_Y', 'Des_CF1_Y'])
'''
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


def static_threeD_position():
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
def static_inv_plot():
    df = pd.read_csv(I_joc_path)
    df["rel_time"] = df["time_s"] - df["time_s"].iloc[0]
    t = df["rel_time"].to_numpy()

    # Create figure and subplots
    fig_IJ, (ack11, ack12) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig_IJ.suptitle("Inverse Jacobian Commands to Drones")

    # Shared y-axis limits
    y_min = df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().min() - 0.1
    y_max = df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().max() + 0.1

    # Subplot for CF1 (ack11)
    ack11.plot(t, df["x1dot"], label="x1_dot", color="red")
    ack11.plot(t, df["y1dot"], label="y1_dot", color="green")
    ack11.plot(t, df["z1dot"], label="z1_dot", color="blue")
    ack11.set_ylabel("CF1 Velocity [m/s]")
    ack11.grid(True)
    ack11.legend()

    # Subplot for CF2 (ack12)
    ack12.plot(t, df["x2dot"], label="x2_dot", color="purple")
    ack12.plot(t, df["y2dot"], label="y2_dot", color="brown")
    ack12.plot(t, df["z2dot"], label="z2_dot", color="orange")
    ack12.set_ylabel("CF2 Velocity [m/s]")
    ack12.set_xlabel("Time [s]")
    ack12.grid(True)
    ack12.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()



# --------------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------          ANIMATIONS             ---------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------------

#--------------------------
#| Animation for 3D Graph |
#--------------------------
# 3 graph for the position of the cluster
def anim_threeD_Plot():
    fig = plt.figure("3D Positions Over Time")
    ax = fig.add_subplot(111, projection="3d")

    # compute bounds as before…
    all_positions = np.vstack([
        cur_cf1_positions,
        cur_cf2_positions,
        cur_cluster_positions,
        des_cluster_positions,
        des_cf1_positions,
        des_cf2_positions,
    ])
    padding = 0.05
    x_min, x_max = all_positions[:,0].min(), all_positions[:,0].max()
    x_range = x_max - x_min
    ax.set_xlim(x_min - padding*x_range, x_max + padding*x_range)
    ax.set_ylim(0,2)
    ax.set_zlim(0,1)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Z Position')
    ax.set_zlabel('Y Position')

    # datasets with their colors
    datasets = {
        'Cur_CF1': (cur_cf1_positions, 'r'),
        'Cur_CF2': (cur_cf2_positions, 'g'),
        'Cur_Cluster': (cur_cluster_positions, 'gray'),
        'Des_Cluster': (des_cluster_positions, 'purple'),
        'Des_CF1': (des_cf1_positions, 'm'),
        'Des_CF2': (des_cf2_positions, 'c'),
    }

    trail_scatters = {}
    head_scatters = {}
    for key, (pos, color) in datasets.items():
        if key.startswith('Des_Cluster'):
            trail_scatters[key] = ax.scatter([], [], [], c=color, s=80, alpha=0.6)
        elif key.startswith('Des_'):
            trail_scatters[key] = ax.scatter([], [], [], c=color, s=30, alpha=0.6)
        else:
            trail_scatters[key] = ax.scatter([], [], [], c=color, s=10, alpha=0.6)
        if key.startswith('Cur_'):
            head_scatters[key] = ax.scatter([], [], [], c='k', s=60)

    # ax.legend(datasets.keys(), loc='lower left')
    handles = [trail_scatters[key] for key in datasets]
    labels  = list(datasets.keys())

    ax.legend(handles, labels,
            loc='lower left',
            #   bbox_to_anchor=(1.05, 1),
            borderaxespad=0.)

    plt.title('3D Position Data')

    def update(frame):
        artists = []
        for key, (pos, color) in datasets.items():
            # update trail
            xs, ys, zs = pos[:frame+1,0], pos[:frame+1,1], pos[:frame+1,2]
            trail_scatters[key]._offsets3d = (xs, ys, zs)
            artists.append(trail_scatters[key])
            # update head only if it exists
            if key in head_scatters:
                xh, yh, zh = pos[frame,0], pos[frame,1], pos[frame,2]
                head_scatters[key]._offsets3d = ([xh], [yh], [zh])
                artists.append(head_scatters[key])
        return artists

    num_frames = min(len(p) for p, _ in datasets.values())
    ani = FuncAnimation(fig, update, frames=num_frames, interval=100, blit=False)

#--------------------------
#| Animation for 2D Graph |
#--------------------------
# 2d inverse jacobian graph for the cluster
def anim_2d_plot():

    df = pd.read_csv(I_joc_path)
    df["rel_time"] = df["time_s"] - df["time_s"].iloc[0]
    
    # reads both csv files for 3d and 2d graphs to calculate number of lines
    with open(file_path, 'r') as f:
        threed_lines = sum(1 for line in f)

    with open(I_joc_path, 'r') as f:
        twod_lines = sum(1 for line in f)

    # calculates elapsed time to match timing of both graphs simultaniously when being animated
    step = twod_lines/threed_lines
    step = int(step)
    data['Timestamp'] = pd.to_datetime(data['Timestamp'])
    start_time = data['Timestamp'].iloc[0]
    end_time = data['Timestamp'].iloc[-1]
    elapsed_seconds = (end_time - start_time).total_seconds()
    print(elapsed_seconds)
    t2 = np.linspace(0, elapsed_seconds, len(df["x1dot"].to_numpy()[::step]))

    # downsampled time and signals
    #t2 = df["rel_time"].to_numpy()[::step]
    x1dot = df["x1dot"].to_numpy()[::step]
    x2dot = df["x2dot"].to_numpy()[::step]
    y1dot = df["y1dot"].to_numpy()[::step]
    y2dot = df["y2dot"].to_numpy()[::step]
    z1dot = df["z1dot"].to_numpy()[::step]
    z2dot = df["z2dot"].to_numpy()[::step]

    # create animated figure
    fig_IJ, (ack1, ack2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig_IJ.suptitle("Inverse Jacobian Commands to Drones")

    # shared axis limits
    y_min = df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().min() - 0.1
    y_max = df[["x1dot", "x2dot", "y1dot", "y2dot", "z1dot", "z2dot"]].to_numpy().max() + 0.1

    # plot 1: cf1
    ack1.set_ylabel("velocity [m/s]")
    ack1.set_xlabel("Time [s]")
    ack1.set_xlim(t2[0], t2[-1])
    ack1.set_ylim(y_min, y_max)
    ack1.grid(True)
    (line_x1,) = ack1.plot([], [], label="x1dot", color='red')
    (line_y1,) = ack1.plot([], [], label="y1dot", color='green')
    (line_z1,) = ack1.plot([], [], label="z1dot", color='blue')
    ack1.legend()

    # plot 2: cf2
    ack2.set_ylabel("z velocity [m/s]")
    ack2.set_xlabel("Time [s]")
    ack2.set_xlim(t2[0], t2[-1])
    ack2.set_ylim(y_min, y_max)
    ack2.grid(True)
    (line_x2,) = ack2.plot([], [], label="x2dot", color='purple')
    (line_y2,) = ack2.plot([], [], label="y2dot", color='brown')
    (line_z2,) = ack2.plot([], [], label="z2dot", color='green')
    ack2.legend()

    fig_IJ.tight_layout(rect=[0, 0.03, 1, 0.95])  # leave space for suptitle

    # update function for animation
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

# we are using this to tune cf2 by itself
def cf2_tuning_static():

    data = pd.read_csv(cf2_path)

    # Convert the 'Timestamp' column to datetime
    #data['time_s'] = pd.to_datetime(data['time_s'])

    # Ensure the data columns are numeric and handle any potential issues with missing or invalid data
    data['x'] = pd.to_numeric(data['x'], errors='coerce')
    data['y'] = pd.to_numeric(data['y'], errors='coerce')
    data['z'] = pd.to_numeric(data['z'], errors='coerce')

    x = data['x'].to_numpy()
    y = data['y'].to_numpy()
    z = data['z'].to_numpy()
   
    # Drop rows with NaN values in the relevant columns
    data = data.dropna(subset=['time_s', 'x', 'y','z'])



    fig = plt.figure("3D Positions cf2 testing")
    ax = fig.add_subplot(111, projection="3d")

    # Plot each dataset
    # ax.scatter(cur_cf1_positions[:, 0], cur_cf1_positions[:, 1], cur_cf1_positions[:, 2], label='Cur_CF1', c='r')
    ax.scatter(x, z, y, label='CF2_VALUES', c='r')

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
    plt.show()

cf2_tuning_static()