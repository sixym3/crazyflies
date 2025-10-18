import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D

# Column names for logandfly.py output (8 columns)
column_names = [
    'range.front', 'range.back', 'range.up', 'range.left',
    'range.right', 'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'
]

# Find latest CSV file
log_dir = 'logs'
if os.path.exists(log_dir):
    csv_files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]
    if csv_files:
        latest_file = max(csv_files, key=lambda x: os.path.getctime(os.path.join(log_dir, x)))
        csv_path = os.path.join(log_dir, latest_file)
        print(f"Loading: {latest_file}")
    else:
        print("No CSV files found")
        exit()
else:
    print("Logs directory not found")
    exit()

# Load data
data = pd.read_csv(csv_path, header=None, names=column_names)
data_cleaned = data[(data != 0).any(axis=1)]
print(f"Data points: {len(data_cleaned)}")

# Create time index (40ms intervals from logandfly.py)
time_index = np.arange(len(data_cleaned)) * 0.04
data_cleaned.index = time_index

# Create plots
fig = plt.figure(figsize=(15, 10))
fig.suptitle(f'Crazyflie Flight Analysis - {os.path.basename(csv_path)}', fontsize=16)

# 3D trajectory plot
ax1 = fig.add_subplot(221, projection='3d')
x = data_cleaned['stateEstimate.x']
y = data_cleaned['stateEstimate.y']
z = data_cleaned['stateEstimate.z']

ax1.plot(x, y, z, 'b-', linewidth=2, alpha=0.8)
ax1.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', s=100, label='Start')
ax1.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', s=100, label='End')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Flight Path')
ax1.legend()

# Range sensors over time
ax2 = fig.add_subplot(222)
range_cols = ['range.front', 'range.back', 'range.up', 'range.left', 'range.right']
for col in range_cols:
    ax2.plot(time_index, data_cleaned[col], label=col, linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Distance (mm)')
ax2.set_title('Range Sensors')
ax2.legend()
ax2.grid(True, alpha=0.3)

# XY trajectory (top view)
ax3 = fig.add_subplot(223)
ax3.plot(x, y, 'b-', linewidth=2)
ax3.scatter(x.iloc[0], y.iloc[0], color='green', s=100, label='Start')
ax3.scatter(x.iloc[-1], y.iloc[-1], color='red', s=100, label='End')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_title('Top View (XY)')
ax3.legend()
ax3.grid(True, alpha=0.3)
ax3.axis('equal')

# Position over time
ax4 = fig.add_subplot(224)
ax4.plot(time_index, x, label='X', linewidth=2)
ax4.plot(time_index, y, label='Y', linewidth=2)
ax4.plot(time_index, z, label='Z', linewidth=2)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Position (m)')
ax4.set_title('Position vs Time')
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Flight analysis
total_distance = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))
print(f"\nFlight Analysis:")
print(f"Duration: {time_index[-1]:.1f}s")
print(f"Total distance: {total_distance:.2f}m")
print(f"Max altitude: {z.max():.2f}m")
print(f"Flight area: {x.max()-x.min():.2f}m × {y.max()-y.min():.2f}m")