"""
Occupancy Grid for obstacle mapping and path planning

Builds a 2D occupancy grid from sensor data for A* path planning.
"""

import math
import numpy as np

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from config import *


class OccupancyGrid:
    """2D occupancy grid built from sensor data."""

    def __init__(self, resolution=GRID_RESOLUTION, size=GRID_SIZE):
        """
        Initialize occupancy grid.

        Args:
            resolution: Grid resolution in meters per cell
            size: Grid size in meters (square grid)
        """
        self.resolution = resolution
        self.size = size
        self.cells_per_side = int(size / resolution)
        self.grid = np.zeros((self.cells_per_side, self.cells_per_side), dtype=np.uint8)
        self.center_x = self.cells_per_side // 2
        self.center_y = self.cells_per_side // 2
        self.drone_pos = (self.center_x, self.center_y)
        self.safety_margin = OBSTACLE_INFLATION

        # Persistent obstacle map in world coordinates
        self.persistent_obstacles = set()  # Set of (world_x, world_y) tuples
        self.last_drone_world_pos = (0.0, 0.0)  # Track drone world position

    def world_to_grid(self, x, y, drone_x=0, drone_y=0):
        """Convert world coordinates to grid coordinates relative to drone."""
        rel_x = x - drone_x
        rel_y = y - drone_y
        grid_x = int(self.center_x + rel_x / self.resolution)
        grid_y = int(self.center_y + rel_y / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y, drone_x=0, drone_y=0):
        """Convert grid coordinates to world coordinates."""
        rel_x = (grid_x - self.center_x) * self.resolution
        rel_y = (grid_y - self.center_y) * self.resolution
        world_x = drone_x + rel_x
        world_y = drone_y + rel_y
        return world_x, world_y

    def is_valid_cell(self, x, y):
        """Check if grid coordinates are within bounds."""
        return 0 <= x < self.cells_per_side and 0 <= y < self.cells_per_side

    def add_obstacle_points(self, points, drone_position):
        """Add obstacle points from sensor data to grid with persistent mapping."""
        drone_x, drone_y = drone_position[0], drone_position[1]
        self.last_drone_world_pos = (drone_x, drone_y)

        # Add new obstacles to persistent map
        new_obstacles_count = 0
        total_count = len(points)
        obstacle_range = 2.5  # meters

        for point in points:
            if len(point) >= 2:
                # Calculate distance from drone to this obstacle point
                dx = point[0] - drone_x
                dy = point[1] - drone_y
                distance = math.sqrt(dx * dx + dy * dy)

                # Add obstacles within reasonable distance to persistent map
                if distance <= obstacle_range:
                    # Round to grid resolution for consistent storage
                    world_x = round(point[0] / self.resolution) * self.resolution
                    world_y = round(point[1] / self.resolution) * self.resolution

                    if (world_x, world_y) not in self.persistent_obstacles:
                        self.persistent_obstacles.add((world_x, world_y))
                        new_obstacles_count += 1

        # Rebuild grid from persistent obstacles
        self._rebuild_grid_from_persistent_obstacles(drone_x, drone_y)

        return new_obstacles_count, total_count

    def _rebuild_grid_from_persistent_obstacles(self, drone_x, drone_y):
        """Rebuild the grid from persistent obstacles centered on current drone position."""
        # Clear grid
        self.grid.fill(0)

        # Add all persistent obstacles that are within the current grid view
        nearby_count = 0
        view_range = (self.size / 2) + 1.0  # Add buffer for grid boundary

        for world_x, world_y in self.persistent_obstacles:
            # Check if obstacle is within current grid view
            dx = world_x - drone_x
            dy = world_y - drone_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance <= view_range:
                grid_x, grid_y = self.world_to_grid(world_x, world_y, drone_x, drone_y)
                if self.is_valid_cell(grid_x, grid_y):
                    self.grid[grid_y, grid_x] = 1
                    nearby_count += 1

        # Inflate obstacles for safety margin
        self._inflate_obstacles()

        # Mark drone position as free and update drone grid position
        self.drone_pos = (self.center_x, self.center_y)
        self.grid[self.center_y, self.center_x] = 0

        return nearby_count

    def _inflate_obstacles(self):
        """Inflate obstacles by safety margin."""
        if self.safety_margin <= 0:
            return

        # Simple dilation (inflate obstacles)
        inflated = np.zeros_like(self.grid)
        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                if self.grid[y, x] == 1:
                    # Inflate around this obstacle
                    for dy in range(-self.safety_margin, self.safety_margin + 1):
                        for dx in range(-self.safety_margin, self.safety_margin + 1):
                            ny, nx = y + dy, x + dx
                            if self.is_valid_cell(nx, ny):
                                inflated[ny, nx] = 1

        self.grid = inflated

    def get_target_position(self, distance=TARGET_DISTANCE, angle=0.0):
        """Get target position in grid coordinates."""
        target_x = distance * math.cos(math.radians(angle))
        target_y = distance * math.sin(math.radians(angle))
        grid_x, grid_y = self.world_to_grid(target_x, target_y)

        # Ensure target is within grid bounds (but don't clamp for relative targets)
        return grid_x, grid_y

    def get_obstacles_count(self):
        """Get number of obstacle cells in current grid view."""
        return np.sum(self.grid == 1)

    def get_persistent_obstacles_count(self):
        """Get total number of persistent obstacles discovered."""
        return len(self.persistent_obstacles)

    def get_grid_stats(self):
        """Get grid statistics."""
        total_cells = self.grid.size
        obstacle_cells = self.get_obstacles_count()
        free_cells = total_cells - obstacle_cells
        persistent_obstacles = self.get_persistent_obstacles_count()
        return {
            'total': total_cells,
            'obstacles': obstacle_cells,
            'free': free_cells,
            'obstacle_percentage': 100 * obstacle_cells / total_cells,
            'persistent_obstacles': persistent_obstacles
        }