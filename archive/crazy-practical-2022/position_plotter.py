import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

# Column names for logandfly.py output
column_names = [
    'range.front', 'range.back', 'range.up', 'range.left',
    'range.right', 'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'
]

def find_latest_log():
    """Find the latest CSV log file"""
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        print("Logs directory not found")
        return None

    csv_files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]
    if not csv_files:
        print("No CSV files found")
        return None

    latest_file = max(csv_files, key=lambda x: os.path.getctime(os.path.join(log_dir, x)))
    return os.path.join(log_dir, latest_file)

def load_and_process_data(csv_path):
    """Load and clean the log data"""
    data = pd.read_csv(csv_path, header=None, names=column_names)
    data_cleaned = data[(data != 0).any(axis=1)]

    # Create time index (40ms intervals from logandfly.py)
    time_index = np.arange(len(data_cleaned)) * 0.04
    data_cleaned.index = time_index

    return data_cleaned, time_index

def plot_drone_position(data, time_index, filename):
    """Create 2D bird's eye view and height vs time plots"""

    # Extract position data
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle(f'Drone Position Analysis - {os.path.basename(filename)}', fontsize=16)

    # 2D Bird's Eye View (XY plane)
    ax1.plot(x, y, 'b-', linewidth=2, alpha=0.8, label='Flight path')
    ax1.scatter(x.iloc[0], y.iloc[0], color='green', s=150, marker='o',
                label='Start', zorder=5, edgecolors='black', linewidth=2)
    ax1.scatter(x.iloc[-1], y.iloc[-1], color='red', s=150, marker='s',
                label='End', zorder=5, edgecolors='black', linewidth=2)

    # Add direction arrows
    n_arrows = 10
    step = len(x) // n_arrows
    for i in range(0, len(x)-step, step):
        dx = x.iloc[i+step] - x.iloc[i]
        dy = y.iloc[i+step] - y.iloc[i]
        ax1.arrow(x.iloc[i], y.iloc[i], dx*0.3, dy*0.3,
                 head_width=0.02, head_length=0.02, fc='orange', ec='orange', alpha=0.7)

    ax1.set_xlabel('X Position (m)', fontsize=12)
    ax1.set_ylabel('Y Position (m)', fontsize=12)
    ax1.set_title('2D Bird\'s Eye View', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Add position annotations
    ax1.annotate(f'Start\n({x.iloc[0]:.2f}, {y.iloc[0]:.2f})',
                xy=(x.iloc[0], y.iloc[0]), xytext=(10, 10),
                textcoords='offset points', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.7))
    ax1.annotate(f'End\n({x.iloc[-1]:.2f}, {y.iloc[-1]:.2f})',
                xy=(x.iloc[-1], y.iloc[-1]), xytext=(10, -20),
                textcoords='offset points', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightcoral', alpha=0.7))

    # Height vs Time
    ax2.plot(time_index, z, 'r-', linewidth=2, label='Height')
    ax2.fill_between(time_index, 0, z, alpha=0.3, color='red')
    ax2.scatter(time_index[0], z.iloc[0], color='green', s=100, zorder=5, label='Start')
    ax2.scatter(time_index[-1], z.iloc[-1], color='red', s=100, zorder=5, label='End')

    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Height (m)', fontsize=12)
    ax2.set_title('Height vs Time', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    # Add statistics text
    max_height = z.max()
    min_height = z.min()
    avg_height = z.mean()
    flight_duration = time_index[-1]

    stats_text = f'Max: {max_height:.2f}m\nMin: {min_height:.2f}m\nAvg: {avg_height:.2f}m\nDuration: {flight_duration:.1f}s'
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()

    # Print flight analysis
    total_distance_2d = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    total_distance_3d = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))

    print(f"\nFlight Analysis:")
    print(f"Duration: {flight_duration:.1f}s")
    print(f"2D distance traveled: {total_distance_2d:.2f}m")
    print(f"3D distance traveled: {total_distance_3d:.2f}m")
    print(f"Height - Max: {max_height:.2f}m, Min: {min_height:.2f}m, Avg: {avg_height:.2f}m")
    print(f"Flight area: {x.max()-x.min():.2f}m × {y.max()-y.min():.2f}m")

    return fig

def main():
    # Find latest log file
    csv_path = find_latest_log()
    if csv_path is None:
        return

    print(f"Loading: {os.path.basename(csv_path)}")

    # Load and process data
    data, time_index = load_and_process_data(csv_path)
    print(f"Data points: {len(data)}")

    # Create plots
    fig = plot_drone_position(data, time_index, csv_path)

    # Show plots
    plt.show()

if __name__ == '__main__':
    main()