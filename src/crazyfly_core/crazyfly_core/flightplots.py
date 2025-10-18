import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pathlib  
import subprocess, shlex
import os
import time
import numpy as np
from matplotlib.animation import FuncAnimation
import math
import glob

# --------------------------------------------------------------------------------------------------------------------------------
# ------------------------------        AUTOREADER FOR POSITION & INVERSE JACOBIAN DATA    --------------- 07/04/25 -------------
# --------------------------------------------------------------------------------------------------------------------------------

def FILE_INITIATION(i):

    # WORKSPACE   = pathlib.Path(__file__).resolve().parents[3] #3rd parent up is just the crazyfly_ws where cluster_data is --> ~/crazyfly_ws
    # WORKSPACE2 = pathlib.Path(__file__).resolve().parents[3]
    # WORKSPACE3  = pathlib.Path(__file__).resolve().parents[3]
    WORKSPACE = pathlib.Path.home() / "crazyfly_ws"   # <— fixed
    DATA_DIR      = WORKSPACE / "cluster_data"
    INV_DATA_DIR  = WORKSPACE / "I_Joc_values"
    CF2_TUNING    = WORKSPACE / "cf2_tuning_flight_data"
    CF2_RESULTS = WORKSPACE / "flight_navigation_precision/cf2_dynamic_hover"
    CF1_TUNING = WORKSPACE / "flight_navigation_precision/cf1_multiple_waypoint"
    CF1_RESULTS = WORKSPACE / "flight_navigation_precision/cf1_dynamic_hover"
    CF1_COMMANDS = WORKSPACE / "flight_navigation_precision/command_values_for_stability/cf1_command_values"
    CF2_QUAT = WORKSPACE / "cf2_quat_values"


    # DATA_DIR    = WORKSPACE / "cluster_data"# <— save_data_to_csv() writes here
    # INV_DATA_DIR = WORKSPACE2 / "I_Joc_values"  
    # CF2_TUNING = WORKSPACE3 / "cf2_tuning"     

    PATTERN     = "cluster_data_*.csv"                             # matches all cluster logs
    FILE_PATTERN = "cluster_dot_*.csv"
    FILE_PATTERN2 = "cf2_tuning_*.csv"
    FILE_PATTERN3 = "cf1_tuning_*.csv"  
    FILE_PATTERN_CF1_CMD = "cf1_all_values_*.csv"


    csv_files   = sorted(DATA_DIR.glob(PATTERN))                #finds the file through glob and sorted
    I_Joc_files = sorted(INV_DATA_DIR.glob(FILE_PATTERN))
    # cf2_files = sorted(CF2_TUNING.glob(FILE_PATTERN2))
    # cf1_files = sorted(CF1_TUNING.glob(FILE_PATTERN3))
    cf2_files = sorted(CF2_RESULTS.glob(FILE_PATTERN2))
    cf1_files = sorted(CF1_RESULTS.glob(FILE_PATTERN3))
    cf1_cmd_files = sorted(CF1_COMMANDS.glob(FILE_PATTERN_CF1_CMD))

    # ERROR STATEMENT
    if not csv_files:
        raise FileNotFoundError(f"No files matching {PATTERN} in {DATA_DIR}")
    if not I_Joc_files:
        raise FileNotFoundError(f"No file matching {FILE_PATTERN} in {INV_DATA_DIR}")
    if not cf2_files:
        raise FileNotFoundError(f"No file matching {FILE_PATTERN2} in {CF2_TUNING}")
    if not cf1_files:
        raise FileNotFoundError(f"No file matching {FILE_PATTERN3} in {CF1_TUNING}")
    if not cf1_cmd_files:
        raise FileNotFoundError(f"No file matching {FILE_PATTERN_CF1_CMD} in {CF1_COMMANDS}")

    file_path   = csv_files[-1]            # newest because the timestamp sorts lexicographically
    I_joc_path = I_Joc_files[-1]
    cf2_path = cf2_files[-1]
    cf1_path = cf1_files[-1]
    cf1_cmd_path = cf1_cmd_files[-2]

    print(cf1_path)

    if i == "cf1_path":
        return cf1_path
    if i == "cf2_path":
        return cf2_path
    if i == "I_joc_path":
        return I_joc_path
    if i == "file_path":
        return file_path
    if i == "cf1_cmd_path":
        return cf1_cmd_path
    else:
        return cf2_path

def load_position_data(file_path):
    df = pd.read_csv(file_path)
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])
    df = df.dropna(subset=['Timestamp', 'Cur_CF1_Y', 'Des_CF1_Y'])

    cur_cf1_positions = df[['Cur_CF1_X', 'Cur_CF1_Z', 'Cur_CF1_Y']].to_numpy()
    cur_cf2_positions = df[['Cur_CF2_X', 'Cur_CF2_Z', 'Cur_CF2_Y']].to_numpy()
    cur_cluster_positions = df[['Cur_Cluster_X', 'Cur_Cluster_Z', 'Cur_Cluster_Y']].to_numpy()
    des_cluster_positions = df[['Des_Cluster_X', 'Des_Cluster_Z', 'Des_Cluster_Y']].to_numpy()
    des_cf1_positions = df[['Des_CF1_X', 'Des_CF1_Z', 'Des_CF1_Y']].to_numpy()
    des_cf2_positions = df[['Des_CF2_X', 'Des_CF2_Z', 'Des_CF2_Y']].to_numpy()

    return df, cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions


# open file to see data
# subprocess.run(shlex.split(f"code -r {file_path}"))

# --------------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------------

# Filepath to the CSV file
# file_path = "/home/rsl/crazyfly_ws/cluster_data/cluster_data_20250701_154916.csv"
# # Read the CSV file into a DataFrame
#data = pd.read_csv(file_path)



"""#-----------------------------------
data = pd.read_csv(FILE_INITIATION())


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
Cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions = load_position_data(file_path)

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
#-----------------------------------------------
"""
def static_threeD_position(cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions):
    # Plot 3D position data
    fig = plt.figure("3D Positions")
    ax = fig.add_subplot(111, projection="3d")

    # unknown variables resolved by parameter
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
    plt.show()




# --------------------------------- INVERSE JACOBIAN PLOTS -------------------------------------
def static_inv_plot(I_joc_path):
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
def anim_threeD_Plot(cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions):
    fig = plt.figure("3D Positions Over Time")
    ax = fig.add_subplot(111, projection="3d")

    # unknown variables resolved by parameter
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
    #ax.set_xlim(x_min - padding*x_range, x_max + padding*x_range)
    ax.set_xlim(0,6)
    ax.set_ylim(0,6)
    ax.set_zlim(0,3)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Z Position')
    ax.set_zlabel('Y Position')
    ax.set_box_aspect((1,1,1))

    # unknown variables resolved by parameter
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
    plt.show()

#--------------------------
#| Animation for 2D Graph |
#--------------------------
# 2d inverse jacobian graph for the cluster
def anim_2d_plot(I_joc_path, file_path, timestamp_df):

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
    file_path = FILE_INITIATION("file_path")
    data = pd.read_csv(file_path)
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
def cf2_tuning_static(cf2_path):

    #data = pd.read_csv(cf2_path)
    # cf2_path = FILE_INITIATION("cf2_path")
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

#cf2_tuning_static()

def rmse(diff):
    return diff**2

# calculates error of x y and z for each drone in a cluster using RMSE within 0.2m of desired cluster location
# currently hard-coded for simple motion of going up 1 m in the air, holding position, and coming back down
# current goal is to have all errors < 1 cm
def cluster_accuracy():
    file_path = FILE_INITIATION("file_path")
    df = pd.read_csv(file_path)

    df['Err_CF1_X'] = df['Cur_CF1_X'] - df['Des_CF1_X']
    err_values_x = list(df['Err_CF1_X'])
    squared_x = [x**2 for x in err_values_x]
    sum_x = sum(squared_x)
    rmse_x = math.sqrt(sum_x/len(df))

    # since the drone going up and coming back down is not related to the desired cluster location,
    # only values in a 0.2m radius of the desired cluster are accepted
    df['Err_CF1_Y'] = df['Cur_CF1_Y'] - df['Des_CF1_Y']
    df_y = df[df['Err_CF1_Y'].abs() <= 0.02]

    err_values_y = df_y['Err_CF1_Y'].tolist()
    squared_y = [y**2 for y in err_values_y]
    sum_y = sum(squared_y)
    rmse_y = math.sqrt(sum_y/len(err_values_y))

    df['Err_CF1_Z'] = df['Cur_CF1_Z'] - df['Des_CF1_Z']
    err_values_z = list(df['Err_CF1_Z'])
    squared_z = [z**2 for z in err_values_z]
    sum_z = sum(squared_z)
    rmse_z = math.sqrt(sum_z/len(df))

    print("error of cf1 and cf2 based on RMSE")
    print("cf1 error: 0.02 m")
    print("x error: ", f"{rmse_x:.6f}", "m")
    print("y error: ", f"{rmse_y:.6f}", "m")
    print("z error: ", f"{rmse_z:.6f}", "m")
    print("-------")

    df['Err_CF2_X'] = df['Cur_CF2_X'] - df['Des_CF2_X']
    err_values_x2 = list(df['Err_CF2_X'])
    squared_x2 = [x**2 for x in err_values_x2]
    sum_x2 = sum(squared_x2)
    rmse_x2 = math.sqrt(sum_x2/len(df))
    

    # since the drone going up and coming back down is not related to the desired cluster location,
    # only values in a 0.2m radius of the desired cluster are accepted
    df['Err_CF2_Y'] = df['Cur_CF2_Y'] - df['Des_CF2_Y']
    df_y2 = df[df['Err_CF2_Y'].abs() <= 0.02]
    error = 0.02
    while df_y2.empty:
        df_y2 = df[df['Err_CF2_Y'].abs() <= error]
        error += .01

    err_values_y2 = df_y2['Err_CF2_Y'].tolist()
    squared_y2 = [y**2 for y in err_values_y2]
    sum_y2 = sum(squared_y2)
    rmse_y2 = math.sqrt(sum_y2/len(err_values_y2))

    df['Err_CF2_Z'] = df['Cur_CF2_Z'] - df['Des_CF2_Z']
    err_values_z2 = list(df['Err_CF2_Z'])
    squared_z2 = [z**2 for z in err_values_z2]
    sum_z2 = sum(squared_z2)
    rmse_z2 = math.sqrt(sum_z2/len(df))

    print("cf2 error: ", error, "m")
    print("x error: ", f"{rmse_x2:.6f}", "m")    
    print("y error: ", f"{rmse_y2:.6f}", "m")
    print("z error: ", f"{rmse_z2:.6f}", "m")

def plot_pos_err_cmd():
    """
    Load the latest CF2 tuning CSV and plot Y, X, Z vs time
    in a 2x2 layout matching the sample (bottom-right left blank).
    No parameters; everything is resolved internally.
    """
    # --- locate data (file or directory) ---
    path = FILE_INITIATION("cf1_cmd_path")

    # if os.path.isdir(cf2_path):
    #     # Pick newest cf2_tuning_*.csv in the folder
    #     candidates = sorted(
    #         glob.glob(os.path.join(cf2_path, "cf2_tuning_*.csv")),
    #         key=os.path.getmtime
    #     )
    #     if not candidates:
    #         raise FileNotFoundError(f"No cf2_tuning_*.csv files found in {cf2_path}")
    #     csv_path = candidates[-1]
    # else:
    #     # FILE_INITIATION returned a specific file path
    #     csv_path = cf2_path
    #     if not os.path.isfile(csv_path):
    #         raise FileNotFoundError(f"Path is not a file: {csv_path}")

    # --- load and sanitize ---
    data = pd.read_csv(path)
    print(data)

    # Ensure numeric columns
    for c in ("x", "y", "z"):
        if c in data.columns:
            data[c] = pd.to_numeric(data[c], errors="coerce")
        else:
            raise KeyError(f"CSV missing required column '{c}'")

    # Build time axis:
    # Prefer a numeric time column if present; else parse 'time_s' (YYYYMMDD_HHMMSS);
    # if that’s too coarse (repeats), fall back to uniform dt.
    dt_guess = 0.01  # seconds (matches your previous average sample period)
    t = None

    # 1) any numeric time column?
    for col in ("t", "elapsed_s"):
        if col in data.columns and np.issubdtype(data[col].dtype, np.number):
            t = data[col].to_numpy()
            break

    # 2) try to parse 'time_s'
    if t is None and "time_s" in data.columns:
        try:
            ts = pd.to_datetime(data["time_s"], format="%Y%m%d_%H%M%S.%f")
            t = (ts - ts.iloc[0]).dt.total_seconds().to_numpy()
            '''
            caleb: i think adding microseconds should eliminate the need for this
            # if resolution too coarse (all values equal), use index * dt
            if len(t) > 1 and np.allclose(t, t[0]):
                t = np.arange(len(data)) * dt_guess
            # remove this (maybe)
            else:
                t = np.arange(len(data)) * dt_guess
            '''
        except Exception:
            t = np.arange(len(data)) * dt_guess

    # 3) final fallback
    if t is None:
        t = np.arange(len(data)) * dt_guess

    # Drop rows with NaNs in plotted columns (keep time array consistent)
    valid = ~(data[["x", "y", "z"]].isna().any(axis=1))
    t = t[valid.to_numpy()]
    x = data.loc[valid, "x"].to_numpy()
    y = data.loc[valid, "y"].to_numpy()
    z = data.loc[valid, "z"].to_numpy()

    error_x = data.loc[valid, "x_error"].to_numpy()
    error_y = data.loc[valid, "y_error"].to_numpy()
    error_z = data.loc[valid, "z_error"].to_numpy()

    yaw = data.loc[valid, "yaw"].to_numpy()
    pitch = data.loc[valid, "pitch"].to_numpy()
    roll = data.loc[valid, "roll"].to_numpy()

    command_yawrate = data.loc[valid, "yaw_command"].to_numpy()
    command_pitch = data.loc[valid, "pitch_command"].to_numpy()
    command_roll = data.loc[valid, "roll_command"].to_numpy()
    command_thrust = data.loc[valid, "thrust_command"].to_numpy()
    min_thrust = 42000
    max_thrust = 55000

    pitch_cmd_p = data.loc[valid, "pitch_cmd_p"].to_numpy()
    pitch_cmd_i = data.loc[valid, "pitch_cmd_i"].to_numpy()
    pitch_cmd_d = data.loc[valid, "pitch_cmd_d"].to_numpy()
    roll_cmd_p = data.loc[valid, "roll_cmd_p"].to_numpy()
    roll_cmd_i = data.loc[valid, "roll_cmd_i"].to_numpy()
    roll_cmd_d = data.loc[valid, "roll_cmd_d"].to_numpy()
    thrust_cmd_p = data.loc[valid, "thrust_cmd_p"].to_numpy()
    thrust_cmd_i = data.loc[valid, "thrust_cmd_i"].to_numpy()
    thrust_cmd_d = data.loc[valid, "thrust_cmd_d"].to_numpy()

    # --- plotting to match your style ---
    fig, axes = plt.subplots(2, 3, sharex=True)
    # Y (top-left)
    axes[0][0].plot(t, y, "r-", label="Y Position", marker=".", markersize=5)
    axes[0][0].plot(t, error_y, "b-", label="Y Error", marker=".", markersize=5)
    axes[0][0].plot(t, (command_thrust-min_thrust)/(max_thrust-min_thrust), "k-", marker=".", markersize=5, label="Normalized Thrust Cmd")
    axes[0][0].plot(t, (thrust_cmd_p)/(max_thrust-min_thrust),"--", color='0.1', label="Normalized p term")
    axes[0][0].plot(t, (thrust_cmd_i)/(max_thrust-min_thrust), "-.", color='0.4',label="Normalized i term")
    axes[0][0].plot(t, (thrust_cmd_d)/(max_thrust-min_thrust), ":", color='0.7', label="Normalized d term")
    axes[0][0].set_xlabel("Time(s)")
    axes[0][0].set_ylabel("Y Position (m)")
    axes[0][0].set_title("Y Position Over Time")
    axes[0][0].legend()

    # Z (top-right)
    axes[0][1].plot(t, z, "r-", label="Z Position", marker=".", markersize=5)
    axes[0][1].plot(t, error_z, "b-", label="Z Error", marker=".", markersize=5)
    axes[0][1].set_xlabel("Time(s)")
    axes[0][1].set_ylabel("Z Position (m)")
    axes[0][1].set_title("Z Position Over Time")
    axes[0][1].legend()

    # X (top-middle)
    axes[0][2].plot(t, x, "r-", label="X Position", marker=".", markersize=5)
    axes[0][2].plot(t, error_x, "b-", label="X Error", marker=".", markersize=5)
    axes[0][2].set_xlabel("Time(s)")
    axes[0][2].set_ylabel("X Position (m)")
    axes[0][2].set_title("X Position Over Time")
    axes[0][2].legend()

    # yaw (bottom left)
    axes[1][0].plot(t, yaw, "r-", label="Yaw", marker=".", markersize=5)
    axes[1][0].plot(t, command_yawrate, "k-", label="Yaw rate cmd", marker=".", markersize=5)
    axes[1][0].set_xlabel("Time(s)")
    axes[1][0].set_ylabel("Yaw (deg)")
    axes[1][0].set_title("Yaw and Commanded Yaw Rate")
    axes[1][0].legend()

    # pitch (bottom middle)
    axes[1][1].plot(t, pitch, "r-", label="Pitch", marker=".", markersize=5)
    axes[1][1].plot(t, command_pitch, "k-", label="Pitch cmd", marker=".", markersize=5)
    axes[1][1].plot(t, pitch_cmd_p, "--", color='0.1', label='p term')
    axes[1][1].plot(t, pitch_cmd_i, "-.", color='0.4', label='i term')
    axes[1][1].plot(t, pitch_cmd_d, ":", color='0.7', label='d term')
    axes[1][1].set_xlabel("Time(s)")
    axes[1][1].set_ylabel("Pitch (deg)")
    axes[1][1].set_title("Pitch and Commanded Pitch")
    axes[1][1].legend()

    axes[1][2].plot(t, roll, "r-", label="Roll", marker=".", markersize=5)
    axes[1][2].plot(t, command_roll, "k-", label="Roll cmd", marker=".", markersize=5)
    axes[1][2].plot(t, roll_cmd_p, "--", color='0.1', label="p term")
    axes[1][2].plot(t, roll_cmd_i, "-.", color='0.4', label="i term")
    axes[1][2].plot(t, roll_cmd_d, ":", color='0.7', label="d term")
    axes[1][2].set_xlabel("Time(s)")
    axes[1][2].set_ylabel("Roll (deg)")
    axes[1][2].set_title("Roll and Commanded Roll")
    axes[1][2].legend()

    plt.subplots_adjust(hspace=0.18, wspace=0.235, left=0.044)
    plt.show()
    #print(f"Plotted: {os.path.basename(csv_path)}")

def main():
    file_path = FILE_INITIATION("file_path")
    I_joc_path = FILE_INITIATION("I_joc_path")
    cf1_path = FILE_INITIATION("cf1_path")
    cf2_path = FILE_INITIATION("cf2_path")

    df, cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions = load_position_data(file_path)

    # static_threeD_position(cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions)
    # anim_threeD_Plot(cur_cf1_positions, cur_cf2_positions, cur_cluster_positions, des_cluster_positions, des_cf1_positions, des_cf2_positions)
    #static_inv_plot(I_joc_path)
    # anim_2d_plot(I_joc_path, file_path, df)
    # cf2_tuning_static(cf1_path)
    #cluster_accuracy()
    plot_pos_err_cmd()
    # plot_quaternion_data()

if __name__ == "__main__":
    main()