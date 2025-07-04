import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pathlib  
import subprocess, shlex
import os

# ---------------------------           AUTOMATICALLY READS FILE NOW             -------------------------------------------
WORKSPACE   = pathlib.Path(__file__).resolve().parents[3] #3rd parent up is just the crazyfly_ws where cluster_data is --> ~/crazyfly_ws
#  print(f"this is the workspace {WORKSPACE}")    
DATA_DIR    = WORKSPACE / "cluster_data"                       # <— save_data_to_csv() writes here
PATTERN     = "cluster_data_*.csv"                             # matches all cluster logs

csv_files   = sorted(DATA_DIR.glob(PATTERN))                #finds the file through glob and sorted

# ERROR STATEMENT
if not csv_files:
    raise FileNotFoundError(f"No files matching {PATTERN} in {DATA_DIR}")

file_path   = csv_files[-1]            # newest because the timestamp sorts lexicographically
print(f"[flightplots] LATEST FLIGHT: {file_path}")

# open file to see data
# subprocess.run(shlex.split(f"code -r {file_path}"))
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

# # Ensure the data columns are numeric and handle any potential issues with missing or invalid data
# data['Cur_CF1_Y'] = pd.to_numeric(data['Cur_CF1_Y'], errors='coerce')
# data['Des_CF1_Y'] = pd.to_numeric(data['Des_CF1_Y'], errors='coerce')

# # Drop rows with NaN values in the relevant columns
# data = data.dropna(subset=['Timestamp', 'Cur_CF1_Y', 'Des_CF1_Y'])

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

# Convert the relevant columns to numpy arrays for compatibility with matplotlib
cur_cf1_positions = data[['Cur_CF1_X', 'Cur_CF1_Z', 'Cur_CF1_Y']].to_numpy()
cur_cf2_positions = data[['Cur_CF2_X', 'Cur_CF2_Z', 'Cur_CF2_Y']].to_numpy()
cur_cluster_positions = data[['Cur_Cluster_X', 'Cur_Cluster_Z', 'Cur_Cluster_Y']].to_numpy()
des_cluster_positions = data[['Des_Cluster_X', 'Des_Cluster_Z', 'Des_Cluster_Y']].to_numpy()
des_cf1_positions = data[['Des_CF1_X', 'Des_CF1_Z', 'Des_CF1_Y']].to_numpy()
des_cf2_positions = data[['Des_CF2_X', 'Des_CF2_Z', 'Des_CF2_Y']].to_numpy()

# Plot 3D position data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

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






csv_path = os.path.expanduser('~/crazyfly_ws/I_Joc_values/cluster_dot.csv')
df = pd.read_csv("~/crazyfly_ws/I_Joc_values/cluster_dot.csv")   # same path as logger
plt.plot(df.time_s, df.x1dot, label="ẋ₁")
plt.plot(df.time_s, df.y1dot, label="ẏ₁")
plt.plot(df.time_s, df.z1dot, label="ż₁")
plt.plot(df.time_s, df.x2dot, label="ẋ₂")
plt.plot(df.time_s, df.y2dot, label="ẏ₂")
plt.plot(df.time_s, df.z2dot, label="ż₂")
plt.xlabel("Time [s]"); plt.ylabel("Commanded velocity [m/s]")
plt.title("Inverse-Jacobian linear velocity commands")
plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()
