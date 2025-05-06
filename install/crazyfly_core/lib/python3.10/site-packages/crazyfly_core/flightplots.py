import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Filepath to the CSV file
file_path = "/home/rsl/crazyfly_ws/cluster_data/cluster_data_20250502_192149.csv"

# Read the CSV file into a DataFrame
data = pd.read_csv(file_path)

col1 = 'Cur_Cluster_Y'
col2 = 'Des_Cluster_Y'

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

# Plot the time series for columns 2 (Cur_CF1_X) and 8 (Cur_Cluster_X)
plt.figure(figsize=(12, 6))
plt.plot(timestamps, line1, label=col1, color='blue')
plt.plot(timestamps, line2, label=col2, color='orange')

# Add labels, title, and legend
plt.xlabel('Timestamp')
plt.ylabel('Values')
plt.title('Time Series Plot of {col1} and {col2}')
plt.legend()
plt.grid()

# Show the plot
plt.tight_layout()
plt.show()

