#!/usr/bin/env python3
"""
Bird's-eye path plotter for Crazyflie logs with occupancy grid generation.

Features:
- Loads the newest CSV in ./logs by default (or a specific file via --file)
- Cleans data, ensures numeric types, drops empty/all-zero rows
- Plots x vs y with equal aspect ratio
- Marks Start/End; optional arrows to show direction
- Generates occupancy grids from multiranger sensor data using position and yaw
- Filters obstacles within configurable distance (default: 1.0m) for grid generation
- Configurable grid resolution and visualization parameters
- Prints stats: samples, duration (if --hz provided), net drift, total path length,
  average speed, and max radial deviation
- Prints obstacle detection and occupancy grid statistics
- Optional Savitzky–Golay smoothing (--smooth WINDOW POLY)
"""

import argparse
import glob
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

EXPECTED_COLS = ["x","y","z","yaw","front","back","left","right","up"]

def latest_csv(log_dir="logs"):
    files = sorted(glob.glob(os.path.join(log_dir, "*.csv")))
    if not files:
        print("No CSVs found in ./logs")
        sys.exit(1)
    return files[-1]

def read_logs(path):
    # Use header row written by np.savetxt(header=...)
    df = pd.read_csv(path, header=0)

    # If someone saved a header line as the first row (stringy), fix it
    if list(df.columns) == [str(i) for i in range(9)] and isinstance(df.iloc[0,0], str):
        new_header = df.iloc[0]
        df = df[1:]
        df.columns = new_header

    # Force numeric
    df = df.apply(pd.to_numeric, errors='coerce')
    # Drop empty rows
    df = df.dropna(how='all')
    # If columns unnamed but count is 9, assign expected names
    if not set(EXPECTED_COLS).issubset(df.columns) and df.shape[1] == 9:
        df.columns = EXPECTED_COLS

    # Drop rows where x & y are both NaN
    df = df.dropna(subset=["x","y"], how="all")
    # Drop all-zero rows (typical padding)
    df = df.loc[~(df.fillna(0)[["x","y"]].eq(0).all(axis=1))]

    return df.reset_index(drop=True)

def path_stats(x, y, hz=None):
    # Differences for path length
    dx = np.diff(x)
    dy = np.diff(y)
    step = np.hypot(dx, dy)
    total_len = float(np.nansum(step))
    net_drift = float(np.hypot(x[-1]-x[0], y[-1]-y[0]))
    max_radius = float(np.nanmax(np.hypot(x - x[0], y - y[0])))
    avg_speed = None
    duration_s = None

    if hz and hz > 0:
        duration_s = (len(x)-1) / hz
        if duration_s > 0:
            avg_speed = total_len / duration_s
    return {
        "samples": len(x),
        "duration_s": duration_s,
        "total_path_m": total_len,
        "net_drift_m": net_drift,
        "max_radial_dev_m": max_radius,
        "avg_speed_mps": avg_speed,
    }

def maybe_smooth(x, y, window_poly):
    if not window_poly:
        return x, y
    try:
        from math import ceil
        window, poly = window_poly
        window = int(window)
        poly = int(poly)
        # window must be odd and >= poly+2
        if window % 2 == 0:
            window += 1
        if window <= poly:
            window = poly + 3 if (poly + 3) % 2 == 1 else poly + 4
        from scipy.signal import savgol_filter
        xs = savgol_filter(x, window, poly, mode="interp")
        ys = savgol_filter(y, window, poly, mode="interp")
        return xs, ys
    except Exception as e:
        print(f"[warn] smoothing skipped ({e})")
        return x, y

def decimate_for_arrows(x, y, every=20):
    idx = np.arange(0, len(x), every)
    return x[idx], y[idx]

def filter_valid_obstacles(ranges, min_range=50, max_range=3000):
    """Filter out invalid range measurements (in mm)."""
    # Convert to numpy array if not already
    ranges = np.array(ranges)
    # Invalid readings: zero, too close, too far, or NaN
    valid = (ranges > min_range) & (ranges < max_range) & ~np.isnan(ranges)
    return valid

def calculate_obstacle_positions(df, min_range=50, max_range=3000):
    """
    Calculate global positions of obstacles detected by multiranger sensors.

    Args:
        df: DataFrame with x, y, yaw, front, back, left, right columns
        min_range: Minimum valid range in mm
        max_range: Maximum valid range in mm

    Returns:
        Dictionary with obstacle positions for each direction
    """
    obstacles = {
        'front': {'x': [], 'y': [], 'distance': []},
        'back': {'x': [], 'y': [], 'distance': []},
        'left': {'x': [], 'y': [], 'distance': []},
        'right': {'x': [], 'y': [], 'distance': []}
    }

    # Sensor angles relative to drone heading (in radians)
    sensor_angles = {
        'front': 0,           # 0 degrees
        'right': np.pi/2,     # 90 degrees
        'back': np.pi,        # 180 degrees
        'left': 3*np.pi/2     # 270 degrees
    }

    for direction in ['front', 'back', 'left', 'right']:
        # Get range measurements for this direction
        ranges = df[direction].to_numpy(dtype=float)

        # Filter valid measurements
        valid_mask = filter_valid_obstacles(ranges, min_range, max_range)

        if not np.any(valid_mask):
            continue

        # Get corresponding position and orientation data
        x_drone = df["x"].to_numpy(dtype=float)[valid_mask]
        y_drone = df["y"].to_numpy(dtype=float)[valid_mask]
        yaw_drone = df["yaw"].to_numpy(dtype=float)[valid_mask]
        ranges_valid = ranges[valid_mask]

        # Convert range from mm to meters
        ranges_m = ranges_valid / 1000.0

        # Calculate global angle for each obstacle
        global_angles = yaw_drone * np.pi/180 + sensor_angles[direction]

        # Calculate obstacle positions in global coordinates
        obstacle_x = x_drone + ranges_m * np.cos(global_angles)
        obstacle_y = y_drone + ranges_m * np.sin(global_angles)

        # Store results
        obstacles[direction]['x'] = obstacle_x
        obstacles[direction]['y'] = obstacle_y
        obstacles[direction]['distance'] = ranges_m

    return obstacles

def calculate_obstacle_stats(obstacles):
    """Calculate statistics about detected obstacles."""
    stats = {}
    total_detections = 0

    for direction in ['front', 'back', 'left', 'right']:
        count = len(obstacles[direction]['x'])
        total_detections += count

        if count > 0:
            distances = obstacles[direction]['distance']
            stats[direction] = {
                'count': count,
                'avg_distance': np.mean(distances),
                'min_distance': np.min(distances),
                'max_distance': np.max(distances)
            }
        else:
            stats[direction] = {
                'count': 0,
                'avg_distance': None,
                'min_distance': None,
                'max_distance': None
            }

    stats['total_detections'] = total_detections
    return stats


def create_occupancy_grid(obstacles, grid_resolution=0.05, padding=0.5, max_distance=1.0):
    """
    Create an occupancy grid from obstacle positions.

    Args:
        obstacles: Dictionary with obstacle positions from calculate_obstacle_positions()
        grid_resolution: Grid cell size in meters (smaller = higher resolution)
        padding: Extra space around obstacles in meters
        max_distance: Maximum distance in meters to include obstacles in grid

    Returns:
        grid: 2D numpy array (occupancy grid)
        extent: [xmin, xmax, ymin, ymax] for matplotlib imshow
        grid_info: Dictionary with grid parameters
    """
    # Collect obstacle positions within distance filter
    all_x = []
    all_y = []
    filtered_count = 0
    total_count = 0

    for direction in ['front', 'back', 'left', 'right']:
        if len(obstacles[direction]['x']) > 0:
            # Filter obstacles by distance
            distances = np.array(obstacles[direction]['distance'])
            valid_mask = distances <= max_distance

            filtered_x = np.array(obstacles[direction]['x'])[valid_mask]
            filtered_y = np.array(obstacles[direction]['y'])[valid_mask]

            all_x.extend(filtered_x)
            all_y.extend(filtered_y)

            filtered_count += np.sum(valid_mask)
            total_count += len(distances)

    if len(all_x) == 0:
        print(f"No obstacles found within {max_distance}m distance filter for grid generation")
        return None, None, None

    print(f"Distance filter: Using {filtered_count}/{total_count} obstacles within {max_distance}m")

    all_x = np.array(all_x)
    all_y = np.array(all_y)

    # Calculate grid bounds with padding
    x_min = np.min(all_x) - padding
    x_max = np.max(all_x) + padding
    y_min = np.min(all_y) - padding
    y_max = np.max(all_y) + padding

    # Calculate grid dimensions
    grid_width = int(np.ceil((x_max - x_min) / grid_resolution))
    grid_height = int(np.ceil((y_max - y_min) / grid_resolution))

    # Initialize grid (0 = free, 1 = occupied)
    grid = np.zeros((grid_height, grid_width), dtype=float)

    # Mark occupied cells
    for x, y in zip(all_x, all_y):
        # Convert world coordinates to grid indices
        grid_x = int((x - x_min) / grid_resolution)
        grid_y = int((y - y_min) / grid_resolution)

        # Ensure indices are within bounds
        if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
            grid[grid_y, grid_x] = 1.0  # Mark as occupied

    # Create extent for matplotlib (note: y-axis is flipped for imshow)
    extent = [x_min, x_max, y_min, y_max]

    grid_info = {
        'resolution': grid_resolution,
        'width': grid_width,
        'height': grid_height,
        'x_min': x_min, 'x_max': x_max,
        'y_min': y_min, 'y_max': y_max,
        'total_cells': grid_width * grid_height,
        'occupied_cells': np.sum(grid > 0),
        'occupancy_ratio': np.sum(grid > 0) / (grid_width * grid_height),
        'max_distance': max_distance,
        'filtered_obstacles': filtered_count,
        'total_obstacles': total_count
    }

    return grid, extent, grid_info

def plot_occupancy_grid(grid, extent, alpha=0.7, colormap='RdYlBu_r'):
    """
    Plot the occupancy grid using matplotlib imshow.

    Args:
        grid: 2D numpy array (occupancy grid)
        extent: [xmin, xmax, ymin, ymax] for matplotlib imshow
        alpha: Transparency of the grid overlay
        colormap: Matplotlib colormap name
    """
    if grid is None:
        return False

    # Create custom colormap: transparent for free space, colored for occupied
    import matplotlib.colors as mcolors

    # Create colormap where 0 is transparent and 1 is red
    colors = ['white', 'red']  # Free space (white), Occupied (red)
    n_bins = 2
    cmap = mcolors.LinearSegmentedColormap.from_list('occupancy', colors, N=n_bins)

    # Plot grid with transparency
    plt.imshow(grid, extent=extent, origin='lower', alpha=alpha, cmap=cmap,
               vmin=0, vmax=1, interpolation='nearest', zorder=2)

    return True

def main():
    ap = argparse.ArgumentParser(description="Plot 2D bird's-eye path of Crazyflie flight with occupancy grid.")
    ap.add_argument("--file", "-f", type=str, default=None,
                    help="CSV file to load (defaults to newest in ./logs)")
    ap.add_argument("--hz", type=float, default=None,
                    help="Logging frequency in Hz (for time-based stats). If you used 100 ms period, set --hz 10.")
    ap.add_argument("--smooth", nargs=2, type=int, metavar=("WINDOW","POLY"),
                    help="Apply Savitzky–Golay smoothing (odd WINDOW, POLY order). Example: --smooth 21 3")
    ap.add_argument("--arrows-every", type=int, default=0,
                    help="If >0, draws small arrows every N samples to indicate direction.")
    ap.add_argument("--xlim", nargs=2, type=float, help="Set x limits: --xlim -1 2")
    ap.add_argument("--ylim", nargs=2, type=float, help="Set y limits: --ylim -1 2")
    ap.add_argument("--no-obstacles", action="store_true",
                    help="Disable obstacle detection and grid generation")
    ap.add_argument("--min-range", type=float, default=50,
                    help="Minimum valid obstacle range in mm (default: 50)")
    ap.add_argument("--max-range", type=float, default=3000,
                    help="Maximum valid obstacle range in mm (default: 3000)")
    ap.add_argument("--max-distance", type=float, default=1.0,
                    help="Maximum distance in meters to include obstacles in grid (default: 1.0)")
    ap.add_argument("--grid-resolution", type=float, default=0.05,
                    help="Grid cell size in meters (default: 0.05)")
    ap.add_argument("--grid-padding", type=float, default=0.5,
                    help="Extra space around obstacles in grid (default: 0.5)")
    ap.add_argument("--grid-alpha", type=float, default=0.7,
                    help="Transparency of occupancy grid overlay (default: 0.7)")
    args = ap.parse_args()

    path = args.file or latest_csv("logs")
    print(f"Loading: {path}")
    df = read_logs(path)

    # Extract x,y
    x = df["x"].to_numpy(dtype=float)
    y = df["y"].to_numpy(dtype=float)

    # Optional smoothing
    x_plot, y_plot = maybe_smooth(x, y, args.smooth)

    # Calculate obstacles if enabled (enabled by default, disabled with --no-obstacles)
    enable_obstacles = not args.no_obstacles
    obstacles = None
    obstacle_stats = None

    if enable_obstacles:
        print("Calculating obstacle positions...")
        obstacles = calculate_obstacle_positions(df,
                                               min_range=args.min_range,
                                               max_range=args.max_range)
        obstacle_stats = calculate_obstacle_stats(obstacles)

        # Initialize visualization variables
        obstacles_plotted = False
        grid_plotted = False
        occupancy_grid = None
        grid_info = None
        extent = None

        # Generate occupancy grid (always generate for obstacle visualization)
        print("Generating occupancy grid...")
        occupancy_grid, extent, grid_info = create_occupancy_grid(
            obstacles,
            grid_resolution=args.grid_resolution,
            padding=args.grid_padding,
            max_distance=args.max_distance
        )
        if occupancy_grid is not None:
            grid_plotted = True
    else:
        # Initialize variables when obstacles are disabled
        obstacles_plotted = False
        grid_plotted = False
        occupancy_grid = None
        grid_info = None
        extent = None

    # Stats
    stats = path_stats(x_plot, y_plot, hz=args.hz)
    print("=== Path Stats ===")
    for k, v in stats.items():
        if v is None:
            print(f"{k}: n/a")
        else:
            if k.endswith("_m") or k.endswith("_mps"):
                print(f"{k}: {v:.3f}")
            elif k.endswith("_s"):
                print(f"{k}: {v:.2f}")
            else:
                print(f"{k}: {v}")

    # Print obstacle statistics if enabled
    if enable_obstacles and obstacle_stats:
        print("\n=== Obstacle Detection Stats ===")
        print(f"Total detections: {obstacle_stats['total_detections']}")
        for direction in ['front', 'back', 'left', 'right']:
            dir_stats = obstacle_stats[direction]
            if dir_stats['count'] > 0:
                print(f"{direction.title()}: {dir_stats['count']} detections, "
                      f"avg: {dir_stats['avg_distance']:.2f}m, "
                      f"range: {dir_stats['min_distance']:.2f}-{dir_stats['max_distance']:.2f}m")
            else:
                print(f"{direction.title()}: no detections")

    # Print occupancy grid statistics if generated
    if grid_plotted and grid_info:
        print("\n=== Occupancy Grid Stats ===")
        print(f"Distance filter: {grid_info['filtered_obstacles']}/{grid_info['total_obstacles']} obstacles within {grid_info['max_distance']:.1f}m")
        print(f"Grid resolution: {grid_info['resolution']:.3f} m/cell")
        print(f"Grid dimensions: {grid_info['width']} x {grid_info['height']} cells")
        print(f"Total cells: {grid_info['total_cells']}")
        print(f"Occupied cells: {grid_info['occupied_cells']}")
        print(f"Occupancy ratio: {grid_info['occupancy_ratio']:.1%}")
        print(f"Covered area: {(grid_info['x_max'] - grid_info['x_min']):.2f} x {(grid_info['y_max'] - grid_info['y_min']):.2f} m")

    # Plot
    plt.figure(figsize=(12, 10))
    plt.plot(x_plot, y_plot, linewidth=1.5, label="Flight Path", zorder=5)
    plt.scatter([x_plot[0]], [y_plot[0]], s=50, marker="o", label="Start", zorder=10)
    plt.scatter([x_plot[-1]], [y_plot[-1]], s=50, marker="x", label="End", zorder=10)

    # Plot occupancy grid if enabled and generated
    if enable_obstacles and occupancy_grid is not None:
        grid_plotted = plot_occupancy_grid(occupancy_grid, extent,
                                         alpha=args.grid_alpha)

    # Title includes grid information
    title = "Crazyflie Path (Bird's-eye: X vs Y)"
    if grid_plotted:
        title += " with Occupancy Grid"

    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True, alpha=0.3)

    # Optional direction arrows
    if args.arrows_every and args.arrows_every > 0 and len(x_plot) > 1:
        xa, ya = decimate_for_arrows(x_plot, y_plot, args.arrows_every)
        # Draw tiny arrows along the path direction
        dx = np.diff(xa, prepend=xa[0])
        dy = np.diff(ya, prepend=ya[0])
        plt.quiver(xa, ya, dx, dy, angles='xy', scale_units='xy', scale=1, width=0.003, alpha=0.6)

    # Optional manual limits
    if args.xlim:
        plt.xlim(args.xlim)
    if args.ylim:
        plt.ylim(args.ylim)

    # Position legend and add colorbar for occupancy grid
    if grid_plotted:
        plt.legend()
        # Add colorbar for occupancy grid
        cbar = plt.colorbar(plt.gci(), shrink=0.6, aspect=20)
        cbar.set_label('Occupancy', rotation=270, labelpad=15)
        cbar.set_ticks([0, 1])
        cbar.set_ticklabels(['Free', 'Occupied'])
    else:
        plt.legend()

    plt.show()

if __name__ == "__main__":
    main()
