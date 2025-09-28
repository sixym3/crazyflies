"""
ASCII Grid Visualization

Terminal-based visualization of the occupancy grid, showing drone position,
target, obstacles, and planned path in ASCII format.
"""

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from config import *


class ASCIIGridVisualizer:
    """ASCII terminal visualization for occupancy grid debugging."""

    def __init__(self, enabled=ENABLE_ASCII_GRID):
        self.enabled = enabled
        self.last_update_count = 0
        self.update_interval = ASCII_UPDATE_INTERVAL

    def toggle(self):
        """Toggle visualization on/off."""
        self.enabled = not self.enabled
        print(f"ASCII Grid visualization: {'ENABLED' if self.enabled else 'DISABLED'}")

    def update(self, data):
        """
        Update visualization with new data.

        Args:
            data: Dictionary containing:
                - grid: OccupancyGrid instance
                - drone_pos: Drone position in grid coordinates
                - target_pos: Target position in grid coordinates
                - path: List of path waypoints (or None)
                - obstacles_detected: Number of obstacles detected
        """
        if not self.enabled:
            return

        # Only print every N updates to avoid terminal spam
        self.last_update_count += 1
        if self.last_update_count % self.update_interval != 0:
            return

        occupancy_grid = data['grid']
        drone_pos = data['drone_pos']
        target_pos = data['target_pos']
        path = data.get('path')
        obstacles_detected = data.get('obstacles_detected', 0)

        self.print_grid(occupancy_grid, drone_pos, target_pos, path, obstacles_detected)

    def print_grid(self, occupancy_grid, drone_pos, target_pos, path=None, obstacles_detected=0):
        """Print ASCII grid to terminal."""
        if not self.enabled:
            return

        # Determine current mode for display
        mode_status = "AUTONOMOUS MODE" if path is not None else "MANUAL MODE"
        path_status = f"(Path: {len(path)} waypoints)" if path else "(No path planning)"

        print("\\n" + "="*80)
        print(f"Occupancy Grid ({occupancy_grid.size}m x {occupancy_grid.size}m) - {mode_status} {path_status}")
        print(f"Obstacles detected: {obstacles_detected}")
        print(f"Showing {VIEW_SIZE*2}x{VIEW_SIZE*2} area around drone")
        print("="*80)

        # Create path sets for quick lookup
        path_cells = set()
        path_waypoints = set()
        if path:
            print(f"Visualization: Processing {len(path)} path waypoints")
            for i, p in enumerate(path):
                grid_x, grid_y = int(p[0]), int(p[1])
                path_cells.add((grid_x, grid_y))
                if i < 5:  # Mark first 5 waypoints differently
                    path_waypoints.add((grid_x, grid_y))
                if i < 3:  # Debug first 3 waypoints
                    print(f"  Path waypoint {i}: ({grid_x}, {grid_y})")
        else:
            print("Visualization: No path data to display")

        # Show area around drone
        drone_x, drone_y = int(drone_pos[0]), int(drone_pos[1])
        view_size = VIEW_SIZE

        start_x = max(0, drone_x - view_size)
        end_x = min(occupancy_grid.cells_per_side, drone_x + view_size + 1)
        start_y = max(0, drone_y - view_size)
        end_y = min(occupancy_grid.cells_per_side, drone_y + view_size + 1)

        print(f"View window: X[{start_x}-{end_x-1}], Y[{start_y}-{end_y-1}]")
        print(f"Drone at: ({drone_x}, {drone_y}), Target at: ({int(target_pos[0])}, {int(target_pos[1])})")

        # Check if path waypoints are within view window
        if path:
            visible_waypoints = 0
            for p in path:
                if start_x <= int(p[0]) < end_x and start_y <= int(p[1]) < end_y:
                    visible_waypoints += 1
            print(f"Path waypoints in view: {visible_waypoints}/{len(path)}")

        # Print grid from top to bottom (reverse y for proper orientation)
        for y in range(end_y - 1, start_y - 1, -1):
            row = ""
            for x in range(start_x, end_x):
                # Check what to display at this position
                if (x, y) == (drone_x, drone_y):
                    row += "D"  # Drone
                elif (x, y) == (int(target_pos[0]), int(target_pos[1])):
                    row += "T"  # Target
                elif (x, y) in path_waypoints:
                    row += "W"  # First 5 waypoints
                elif (x, y) in path_cells:
                    row += "*"  # Path
                elif occupancy_grid.grid[y, x] == 1:
                    row += "#"  # Obstacle
                else:
                    row += "."  # Free space
            print(row)

        print("\\nLegend: D=Drone, T=Target, W=Next_Waypoints, *=Path, #=Obstacle, .=Free")
        print(f"Drone: ({drone_pos[0]}, {drone_pos[1]}), Target: ({target_pos[0]}, {target_pos[1]})")

        if path is not None:
            if len(path) > 0:
                print(f"Autonomous Path: {len(path)} waypoints to target")
            else:
                print("Autonomous Mode: No path found to target")
        else:
            print("Manual Mode: Grid tracking only (no path planning)")

        grid_stats = occupancy_grid.get_grid_stats()
        print(f"Grid: {grid_stats['obstacles']}/{grid_stats['total']} cells occupied ({grid_stats['obstacle_percentage']:.1f}%)")
        print(f"View: X[{start_x}-{end_x-1}], Y[{start_y}-{end_y-1}] of full {occupancy_grid.cells_per_side}x{occupancy_grid.cells_per_side} grid")
        print("="*80)