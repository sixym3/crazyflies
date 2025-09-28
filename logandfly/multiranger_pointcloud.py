#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Example script that plots the output ranges from the Multiranger and Flow
deck in a 3D plot.

When the application is started the Crazyflie will hover at 0.3 m. The
Crazyflie can then be controlled by using keyboard input:
 * Move by using the arrow keys (left/right/forward/backwards)
 * Adjust the height with w/s (0.1 m for each keypress)
 * Yaw slowly using a/d (CCW/CW)
 * Yaw fast using z/x (CCW/CW)
 * Toggle autonomous mode with M (autonomous navigation with obstacle avoidance)

In autonomous mode, the drone will navigate automatically using accumulated
pointcloud data for intelligent path planning to a target 2m ahead. The system:
 * Builds a 2D occupancy grid from 3D pointcloud data
 * Uses A* algorithm for optimal path planning
 * Follows waypoints with smooth movement commands
 * Manual controls override autonomous commands instantly

Requires: pip install pyastar2d

There's additional setting for (see constants below):
 * Plotting the downwards sensor
 * Plotting the estimated Crazyflie position
 * Max threshold for sensors
 * Speed factor that set's how fast the Crazyflie moves

The demo is ended by either closing the graph window.

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Flow deck
 * Multiranger deck
"""
import logging
import math
import sys
import threading
from collections import deque

import numpy as np
try:
    import pyastar2d
except ImportError:
    print("Warning: pyastar2d not installed. Install with: pip install pyastar2d")
    pyastar2d = None
# No matplotlib imports needed for ASCII visualization
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import TurntableCamera
from vispy.visuals import transforms

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
try:
    from sip import setapi
    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt5 import QtCore, QtWidgets


class TerminalGridVisualizer:
    """ASCII terminal visualization for occupancy grid debugging."""

    def __init__(self, enabled=False):
        self.enabled = enabled
        self.last_update_count = 0
        self.update_interval = 10  # Print every 10 updates to avoid spam

    def toggle(self):
        """Toggle visualization on/off."""
        self.enabled = not self.enabled
        print(f"ASCII Grid visualization: {'ENABLED' if self.enabled else 'DISABLED'}")

    def print_grid(self, occupancy_grid, drone_pos, target_pos, path=None):
        """Print ASCII grid to terminal."""
        if not self.enabled:
            return

        # Only print every N updates to avoid terminal spam
        self.last_update_count += 1
        if self.last_update_count % self.update_interval != 0:
            return

        # Determine current mode for display
        mode_status = "AUTONOMOUS MODE" if path is not None else "MANUAL MODE"
        path_status = f"(Path: {len(path)} waypoints)" if path else "(No path planning)"

        print("\n" + "="*80)
        print(f"Occupancy Grid ({occupancy_grid.size}m x {occupancy_grid.size}m) - {mode_status} {path_status}")
        print(f"Showing 40x40 area around drone")
        print("="*80)

        # Create path set for quick lookup
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

        # Show smaller area around drone (40x40 instead of full 100x100)
        drone_x, drone_y = int(drone_pos[0]), int(drone_pos[1])
        view_size = 20  # Show 20 cells in each direction from drone

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

        print("\nLegend: D=Drone, T=Target, W=Next_Waypoints, *=Path, #=Obstacle, .=Free")
        print(f"Drone: ({drone_pos[0]}, {drone_pos[1]}), Target: ({target_pos[0]}, {target_pos[1]})")

        if path is not None:
            if len(path) > 0:
                print(f"Autonomous Path: {len(path)} waypoints to target")
            else:
                print("Autonomous Mode: No path found to target")
        else:
            print("Manual Mode: Grid tracking only (no path planning)")

        print(f"View: X[{start_x}-{end_x-1}], Y[{start_y}-{end_y-1}] of full {occupancy_grid.cells_per_side}x{occupancy_grid.cells_per_side} grid")
        print("="*80)


class OccupancyGrid:
    """2D occupancy grid built from 3D pointcloud data."""

    def __init__(self, resolution=0.1, size=10.0):  # Increased from 6.0 to 10.0
        self.resolution = resolution  # meters per cell
        self.size = size  # grid size in meters (10m x 10m)
        self.cells_per_side = int(size / resolution)  # 60x60 for 0.1m resolution
        self.grid = np.zeros((self.cells_per_side, self.cells_per_side), dtype=np.uint8)
        self.center_x = self.cells_per_side // 2
        self.center_y = self.cells_per_side // 2
        self.drone_pos = (self.center_x, self.center_y)  # Current drone position in grid
        self.safety_margin = 2  # Cells to inflate obstacles

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
        """Add obstacle points from pointcloud to grid."""
        if len(points) == 0:
            return

        # Clear old grid and reset drone position
        self.grid.fill(0)
        drone_x, drone_y = drone_position[0], drone_position[1]

        # Filter and add only nearby pointcloud obstacles to grid
        nearby_count = 0
        total_count = len(points)

        for point in points:
            if len(point) >= 2:  # Ensure we have x, y coordinates
                # Calculate distance from drone to this obstacle point
                dx = point[0] - drone_x
                dy = point[1] - drone_y
                distance = math.sqrt(dx * dx + dy * dy)

                # Add obstacles within reasonable distance for path planning
                obstacle_range = 2.5  # Increased from 1.0m to 2.5m for better obstacle awareness
                if distance <= obstacle_range:
                    grid_x, grid_y = self.world_to_grid(point[0], point[1], drone_x, drone_y)
                    if self.is_valid_cell(grid_x, grid_y):
                        self.grid[grid_y, grid_x] = 1  # Mark as obstacle
                        nearby_count += 1

        print(f"Added obstacles: {nearby_count}/{total_count} points within {obstacle_range}m")

        # Inflate obstacles for safety margin (reduced from 2 to 1 cell)
        self.safety_margin = 1  # 10cm inflation instead of 20cm
        self._inflate_obstacles()

        # Mark drone position as free
        self.drone_pos = (self.center_x, self.center_y)
        self.grid[self.center_y, self.center_x] = 0

    def _inflate_obstacles(self):
        """Inflate obstacles by safety margin."""
        if self.safety_margin <= 0:
            return

        # Create kernel for dilation
        kernel_size = 2 * self.safety_margin + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

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

    def get_target_position(self, distance=2.0, angle=0.0):
        """Get target position in grid coordinates (default 2m forward)."""
        target_x = distance * math.cos(math.radians(angle))
        target_y = distance * math.sin(math.radians(angle))
        grid_x, grid_y = self.world_to_grid(target_x, target_y)

        # Ensure target is within grid bounds
        grid_x = max(0, min(self.cells_per_side - 1, grid_x))
        grid_y = max(0, min(self.cells_per_side - 1, grid_y))

        return grid_x, grid_y


class AsyncPathPlanner:
    """Background thread for A* path planning."""

    def __init__(self):
        self.current_path = deque()
        self.planning_lock = threading.Lock()
        self.should_plan = threading.Event()
        self.stop_planning = threading.Event()
        self.planning_thread = None
        self.grid = None
        self.target_pos = None

    def start(self):
        """Start the planning thread."""
        if self.planning_thread is None or not self.planning_thread.is_alive():
            self.stop_planning.clear()
            self.planning_thread = threading.Thread(target=self._planning_worker)
            self.planning_thread.daemon = True
            self.planning_thread.start()

    def stop(self):
        """Stop the planning thread."""
        self.stop_planning.set()
        if self.planning_thread and self.planning_thread.is_alive():
            self.planning_thread.join(timeout=1.0)

    def request_path(self, grid, target_pos):
        """Request a new path to be computed."""
        with self.planning_lock:
            self.grid = grid
            self.target_pos = target_pos
            self.should_plan.set()

    def get_current_path(self):
        """Get the current planned path (thread-safe)."""
        with self.planning_lock:
            return list(self.current_path)

    def _planning_worker(self):
        """Background worker for path planning."""
        while not self.stop_planning.is_set():
            if self.should_plan.wait(timeout=0.1):
                self.should_plan.clear()

                if pyastar2d is None:
                    continue

                with self.planning_lock:
                    if self.grid is not None and self.target_pos is not None:
                        try:
                            # Prepare grid for A* with distance-based costs
                            # pyastar2d requires: 1+ = traversable, np.inf = impassable
                            weights = np.ones_like(self.grid.grid, dtype=np.float32)  # Base cost 1.0

                            # Add distance-based cost to guide A* toward target
                            goal_row, goal_col = self.target_pos[1], self.target_pos[0]
                            for row in range(weights.shape[0]):
                                for col in range(weights.shape[1]):
                                    if self.grid.grid[row, col] == 0:  # Only for free cells
                                        # Calculate distance to goal
                                        distance = np.sqrt((row - goal_row)**2 + (col - goal_col)**2)
                                        # Cost = base_cost + small_distance_penalty
                                        weights[row, col] = 1.0 + distance * 0.01  # Small distance penalty

                            weights[self.grid.grid == 1] = np.inf  # Obstacles = infinity (impassable)

                            # Validate weight matrix
                            finite_weights = weights[np.isfinite(weights)]
                            free_cells = np.sum(finite_weights >= 1.0)
                            obstacle_cells = np.sum(np.isinf(weights))
                            finite_min = np.min(finite_weights)
                            finite_max = np.max(finite_weights)
                            print(f"Weight matrix: min={finite_min:.3f}, max={finite_max:.3f}, free={free_cells}, obstacles={obstacle_cells}")

                            # Ensure we have valid weights (all finite weights should be >= 1.0)
                            if finite_min < 1.0:
                                print(f"Invalid weight matrix detected! Minimum weight {finite_min} < 1.0")
                                weights = np.ones_like(self.grid.grid, dtype=np.float32)
                                weights[self.grid.grid == 1] = np.inf

                            # Plan path from drone position to target
                            start = (self.grid.drone_pos[1], self.grid.drone_pos[0])  # (row, col)
                            goal = (self.target_pos[1], self.target_pos[0])  # (row, col)

                            print(f"A* planning: start(row,col)={start}, goal(row,col)={goal}")
                            print(f"Grid coords: drone_pos={self.grid.drone_pos}, target_pos={self.target_pos}")

                            # Debug: Show costs around start and goal positions
                            print(f"Cost matrix around start {start}:")
                            for dr in range(-2, 3):
                                row_costs = []
                                for dc in range(-2, 3):
                                    r, c = start[0] + dr, start[1] + dc
                                    if 0 <= r < weights.shape[0] and 0 <= c < weights.shape[1]:
                                        cost = weights[r, c]
                                        if np.isinf(cost):
                                            row_costs.append("INF")
                                        else:
                                            row_costs.append(f"{cost:.2f}")
                                    else:
                                        row_costs.append("OUT")
                                print(f"  {' '.join(row_costs)}")

                            print(f"Cost matrix around goal {goal}:")
                            for dr in range(-2, 3):
                                row_costs = []
                                for dc in range(-2, 3):
                                    r, c = goal[0] + dr, goal[1] + dc
                                    if 0 <= r < weights.shape[0] and 0 <= c < weights.shape[1]:
                                        cost = weights[r, c]
                                        if np.isinf(cost):
                                            row_costs.append("INF")
                                        else:
                                            row_costs.append(f"{cost:.2f}")
                                    else:
                                        row_costs.append("OUT")
                                print(f"  {' '.join(row_costs)}")

                            # Bounds checking for start and goal positions
                            if not (0 <= start[0] < weights.shape[0] and 0 <= start[1] < weights.shape[1]):
                                print(f"Start position {start} out of bounds")
                                self.current_path.clear()
                                continue
                            if not (0 <= goal[0] < weights.shape[0] and 0 <= goal[1] < weights.shape[1]):
                                print(f"Goal position {goal} out of bounds")
                                self.current_path.clear()
                                continue

                            # Ensure start and goal are not in obstacles
                            start_weight = weights[start[0], start[1]]
                            goal_weight = weights[goal[0], goal[1]]
                            start_blocked = np.isinf(start_weight)
                            goal_blocked = np.isinf(goal_weight)
                            print(f"Start weight: {start_weight} (blocked: {start_blocked}), Goal weight: {goal_weight} (blocked: {goal_blocked})")

                            if start_blocked:
                                print(f"Start position {start} is in obstacle, forcing free")
                                weights[start[0], start[1]] = 1.0  # Force start to be free
                            if goal_blocked:
                                print(f"Goal position {goal} is in obstacle, finding nearest free cell")
                                # Find nearest free cell to goal
                                goal = self._find_nearest_free_cell(weights, goal)
                                if goal is None:
                                    print("No free space found near goal")
                                    self.current_path.clear()
                                    continue

                            # Final validation before A*
                            final_start_weight = weights[start[0], start[1]]
                            final_goal_weight = weights[goal[0], goal[1]]
                            if np.isinf(final_start_weight) or np.isinf(final_goal_weight):
                                print(f"Start or goal still in obstacle after correction: start={final_start_weight}, goal={final_goal_weight}")
                                self.current_path.clear()
                                continue

                            # Run A*
                            path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)

                            if path is not None and len(path) > 1:
                                # Convert path back to (x, y) and store
                                self.current_path.clear()
                                print(f"Raw A* path: {path[:5]}...")  # Show first 5 points
                                for i, point in enumerate(path[1:]):  # Skip current position
                                    waypoint = (point[1], point[0])  # Convert (row,col) to (x,y)
                                    self.current_path.append(waypoint)
                                    if i < 3:  # Debug first 3 waypoints
                                        print(f"  Waypoint {i}: (row,col)={point} -> (x,y)={waypoint}")
                                print(f"Path found with {len(path)-1} waypoints")
                            else:
                                print("No path found to target - target may be unreachable")
                                self.current_path.clear()
                                # Try alternative targets if main target is unreachable
                                self._try_alternative_targets(weights, start)

                        except Exception as e:
                            print(f"Path planning error: {e}")
                            self.current_path.clear()

    def _find_nearest_free_cell(self, weights, goal):
        """Find the nearest free cell to the goal position."""
        goal_row, goal_col = goal
        max_search_radius = 10  # Search within 10 cells

        for radius in range(1, max_search_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) == radius or abs(dc) == radius:  # Only check border of current radius
                        new_row = goal_row + dr
                        new_col = goal_col + dc
                        if (0 <= new_row < weights.shape[0] and
                            0 <= new_col < weights.shape[1] and
                            np.isfinite(weights[new_row, new_col]) and
                            weights[new_row, new_col] >= 1.0):
                            print(f"Found free cell at {(new_row, new_col)} near goal {goal}")
                            return (new_row, new_col)
        return None

    def _try_alternative_targets(self, weights, start):
        """Try alternative targets when main target is unreachable."""
        # Try targets at different angles around the drone
        alternative_angles = [45, -45, 90, -90, 135, -135, 180]  # degrees
        target_distance = 1.0  # Try closer targets (1m instead of 2m)

        for angle in alternative_angles:
            # Calculate alternative target position
            angle_rad = math.radians(angle)
            target_x = target_distance * math.cos(angle_rad)
            target_y = target_distance * math.sin(angle_rad)

            # Convert to grid coordinates
            if hasattr(self, 'grid') and self.grid:
                grid_x, grid_y = self.grid.world_to_grid(target_x, target_y)
                if (0 <= grid_x < weights.shape[1] and 0 <= grid_y < weights.shape[0] and
                    weights[grid_y, grid_x] > 0):  # Check if reachable

                    try:
                        alt_path = pyastar2d.astar_path(weights, start, (grid_y, grid_x), allow_diagonal=True)
                        if alt_path is not None and len(alt_path) > 1:
                            self.current_path.clear()
                            for point in alt_path[1:]:
                                self.current_path.append((point[1], point[0]))
                            print(f"Found alternative path at {angle}° with {len(alt_path)-1} waypoints")
                            return
                    except Exception as e:
                        continue  # Try next angle

        print("No alternative paths found - staying in place")


class AutonomousNavigator:
    """Generates navigation commands using pointcloud-based path planning."""

    def __init__(self):
        self.enabled = False
        self.occupancy_grid = OccupancyGrid(resolution=0.1, size=10.0)  # Increased grid size
        self.grid_visualizer = TerminalGridVisualizer(enabled=ENABLE_GRID_VISUALIZATION)
        self.path_planner = AsyncPathPlanner()
        self.current_waypoint = None
        self.waypoint_tolerance = 0.2  # meters
        self.last_drone_position = (0, 0, 0)
        self.pointcloud_data = []
        self.target_world_position = None  # Fixed target in world coordinates

    def set_enabled(self, enabled):
        """Enable or disable autonomous navigation."""
        self.enabled = enabled
        if enabled:
            # Set fixed target 2m forward from current drone position
            current_x, current_y, current_z = self.last_drone_position
            self.target_world_position = (current_x + 2.0, current_y, current_z)
            print(f"Autonomous mode: ENABLED - Target set to {self.target_world_position}")

            # Enable grid visualization for debugging
            self.grid_visualizer.enabled = True
            print("Grid visualization enabled for autonomous mode debugging")

            self.path_planner.start()
            print("Using pointcloud path planning")
        else:
            self.path_planner.stop()
            self.current_waypoint = None
            self.target_world_position = None
            print("Autonomous mode: DISABLED")
        return self.enabled  # Return new state for UI updates

    def update_current_sensors(self, sensor_data, drone_position):
        """Update with current sensor readings (not accumulated pointcloud)."""
        self.last_drone_position = drone_position
        self.current_sensor_data = sensor_data

        # ALWAYS update grid tracking (regardless of autonomous mode)
        self._update_grid_tracking(sensor_data, drone_position)

        # ONLY do autonomous path planning if enabled
        if self.enabled:
            self._do_autonomous_planning()

    def _update_grid_tracking(self, sensor_data, drone_position):
        """Continuously track grid state regardless of autonomous mode."""
        # Create obstacles from current sensor readings only
        current_obstacles = self._create_obstacles_from_sensors(sensor_data, drone_position)

        # Update occupancy grid with current obstacles only
        self.occupancy_grid.add_obstacle_points(current_obstacles, drone_position)

        # Debug output (always show tracking info)
        mode_text = "AUTONOMOUS" if self.enabled else "MANUAL"
        print(f"[{mode_text}] Updated grid: {len(current_obstacles)} obstacles at {drone_position[:2]}")

        # Debug: Show grid statistics
        total_cells = self.occupancy_grid.grid.size
        obstacle_cells = np.sum(self.occupancy_grid.grid == 1)
        print(f"[{mode_text}] Grid stats: {obstacle_cells}/{total_cells} cells occupied ({100*obstacle_cells/total_cells:.1f}%)")

        # Update visualization if enabled (independent of autonomous mode)
        self._update_visualization()

    def _do_autonomous_planning(self):
        """Perform autonomous path planning and navigation."""
        if self.target_world_position is None:
            print("[AUTONOMOUS] No target set - skipping path planning")
            return

        # Convert fixed world target to grid coordinates relative to current drone position
        target_x, target_y = self.target_world_position[0], self.target_world_position[1]
        drone_x, drone_y = self.last_drone_position[0], self.last_drone_position[1]
        target_grid_pos = self.occupancy_grid.world_to_grid(target_x, target_y, drone_x, drone_y)

        self.path_planner.request_path(self.occupancy_grid, target_grid_pos)
        print(f"[AUTONOMOUS] Planning path to fixed target: world={self.target_world_position[:2]} grid={target_grid_pos}")

    def _create_obstacles_from_sensors(self, sensor_data, drone_position):
        """Create obstacle points from current multiranger sensor readings."""
        obstacles = []
        drone_x, drone_y, drone_z = drone_position

        # Get current sensor readings (distances in mm)
        front_dist = sensor_data.get('front', 4000) / 1000.0  # Convert to meters
        back_dist = sensor_data.get('back', 4000) / 1000.0
        left_dist = sensor_data.get('left', 4000) / 1000.0
        right_dist = sensor_data.get('right', 4000) / 1000.0
        up_dist = sensor_data.get('up', 4000) / 1000.0

        # Create obstacles for readings within sensor threshold (2m)
        obstacle_threshold = 2.0  # meters - should match SENSOR_TH/1000

        print(f"Sensor readings: front={front_dist:.2f}m, back={back_dist:.2f}m, left={left_dist:.2f}m, right={right_dist:.2f}m")

        if front_dist < obstacle_threshold:  # Obstacle detected
            # Place obstacle at detected distance and add some depth
            base_pos = [drone_x + front_dist, drone_y, drone_z]
            obstacles.append(base_pos)
            # Add additional obstacle points to represent obstacle thickness
            for offset in [0.1, 0.2]:  # Add points 10cm and 20cm further
                obstacles.append([drone_x + front_dist + offset, drone_y, drone_z])
            print(f"  Front obstacle at {base_pos[:2]} (with depth)")

        if back_dist < obstacle_threshold:
            base_pos = [drone_x - back_dist, drone_y, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x - back_dist - offset, drone_y, drone_z])
            print(f"  Back obstacle at {base_pos[:2]} (with depth)")

        if left_dist < obstacle_threshold:
            base_pos = [drone_x, drone_y + left_dist, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x, drone_y + left_dist + offset, drone_z])
            print(f"  Left obstacle at {base_pos[:2]} (with depth)")

        if right_dist < obstacle_threshold:
            base_pos = [drone_x, drone_y - right_dist, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x, drone_y - right_dist - offset, drone_z])
            print(f"  Right obstacle at {base_pos[:2]} (with depth)")

        # Note: Not using up sensor for 2D navigation

        return obstacles

    def get_navigation_command(self):
        """Generate navigation command using planned path."""
        if not self.enabled:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Get current planned path
        path = self.path_planner.get_current_path()

        if not path:
            # No path available - use simple forward movement as fallback
            if len(self.pointcloud_data) > 0:
                # Check if path ahead is clear using sensor data
                front_clear = self._is_front_clear()
                if front_clear:
                    print("No planned path, but front is clear - moving forward slowly")
                    return {'x': AUTO_SPEED_FACTOR * 0.5, 'y': 0.0, 'yaw': 0.0}  # Slow forward
                else:
                    print("No planned path and obstacles ahead - staying put")
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}  # Stay put

        # Get next waypoint or use current waypoint
        if self.current_waypoint is None and len(path) > 0:
            self.current_waypoint = path[0]

        if self.current_waypoint is None:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Convert waypoint from grid coordinates to world coordinates
        waypoint_world = self.occupancy_grid.grid_to_world(
            self.current_waypoint[0], self.current_waypoint[1],
            self.last_drone_position[0], self.last_drone_position[1]
        )

        # Calculate movement command toward waypoint
        dx = waypoint_world[0] - self.last_drone_position[0]
        dy = waypoint_world[1] - self.last_drone_position[1]
        distance = math.sqrt(dx * dx + dy * dy)

        print(f"Navigation: waypoint_grid={self.current_waypoint}, waypoint_world=({waypoint_world[0]:.3f}, {waypoint_world[1]:.3f})")
        print(f"Navigation: drone_pos=({self.last_drone_position[0]:.3f}, {self.last_drone_position[1]:.3f}), dx={dx:.3f}, dy={dy:.3f}, dist={distance:.3f}")

        # Debug: Also show what world target we're trying to reach
        if self.target_world_position:
            print(f"Navigation: target_world=({self.target_world_position[0]:.3f}, {self.target_world_position[1]:.3f})")

        # Check if we've reached the current waypoint
        if distance < self.waypoint_tolerance:
            print(f"Reached waypoint {self.current_waypoint}, distance: {distance:.2f}m")
            # Move to next waypoint
            if len(path) > 1:
                path.pop(0)  # Remove reached waypoint
                self.current_waypoint = path[0] if path else None
                if self.current_waypoint:
                    print(f"Moving to next waypoint: {self.current_waypoint}")
            else:
                print("Path completed - target reached!")
                self.current_waypoint = None
                return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}  # Path completed

        # Generate movement command
        if distance > 0:
            # Normalize direction and apply speed factor
            cmd_x = (dx / distance) * AUTO_SPEED_FACTOR
            cmd_y = (dy / distance) * AUTO_SPEED_FACTOR

            # Limit maximum speed
            max_speed = AUTO_SPEED_FACTOR
            if abs(cmd_x) > max_speed:
                cmd_x = max_speed if cmd_x > 0 else -max_speed
            if abs(cmd_y) > max_speed:
                cmd_y = max_speed if cmd_y > 0 else -max_speed

            return {'x': cmd_x, 'y': cmd_y, 'yaw': 0.0}
        else:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def _is_front_clear(self, min_distance=1.0):
        """Check if the path ahead is clear using pointcloud data."""
        if not self.pointcloud_data:
            return True  # Assume clear if no data

        # Check for obstacles within min_distance ahead of drone
        drone_x, drone_y = self.last_drone_position[0], self.last_drone_position[1]

        for point in self.pointcloud_data:
            if len(point) >= 2:
                # Check if point is in front of drone within min_distance
                dx = point[0] - drone_x
                dy = point[1] - drone_y
                distance = math.sqrt(dx * dx + dy * dy)

                # Check if obstacle is in front (positive x direction) and close
                if dx > 0 and abs(dy) < 0.5 and distance < min_distance:
                    return False  # Obstacle detected ahead

        return True  # Path ahead appears clear

    def _update_visualization(self):
        """Update the grid visualization with current state."""
        if not self.grid_visualizer.enabled:
            return

        # Get current path (only if in autonomous mode)
        path = self.path_planner.get_current_path() if self.enabled else None

        # Convert coordinates to display coordinates
        drone_grid_pos = self.occupancy_grid.drone_pos

        # Use fixed target if available, otherwise use default
        if self.target_world_position is not None:
            target_x, target_y = self.target_world_position[0], self.target_world_position[1]
            drone_x, drone_y = self.last_drone_position[0], self.last_drone_position[1]
            target_grid_pos = self.occupancy_grid.world_to_grid(target_x, target_y, drone_x, drone_y)
        else:
            target_grid_pos = self.occupancy_grid.get_target_position(distance=2.0, angle=0.0)

        # Update visualization (print ASCII grid to terminal)
        # Show path only in autonomous mode, but always show grid
        self.grid_visualizer.print_grid(
            occupancy_grid=self.occupancy_grid,
            drone_pos=drone_grid_pos,
            target_pos=target_grid_pos,
            path=path  # None in manual mode, actual path in autonomous mode
        )

logging.basicConfig(level=logging.INFO)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = True
# Enable plotting of down sensor
PLOT_SENSOR_DOWN = True
# Set the sensor threashold (in mm)
SENSOR_TH = 2000
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

# Autonomous navigation settings
AUTO_THRESHOLD_MM = 500  # Minimum distance in mm to consider as obstacle
AUTO_SPEED_FACTOR = 0.15  # Speed factor for autonomous movements
AUTO_TURN_RATE = 20  # Turn rate in degrees/second for autonomous mode

# Debug visualization flag - set to True to enable 2D grid plotting
ENABLE_GRID_VISUALIZATION = False  # Change to True to see occupancy grid


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, URI):
        QtWidgets.QMainWindow.__init__(self)

        self.resize(700, 500)
        self.setWindowTitle('Multi-ranger point cloud - Manual Mode (Press M to toggle autonomous)')

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}
        self.autonomous_navigator = AutonomousNavigator()
        self.last_pos = [0.0, 0.0, 0.0]  # Track drone position

        self.canvas = Canvas(self.updateHover, self.autonomous_navigator)
        self.canvas.create_native()
        self.canvas.native.setParent(self)

        self.setCentralWidget(self.canvas.native)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(100)
        self.hoverTimer.start()

    def sendHoverCommand(self):
        # Check if there's manual input (non-zero hover commands)
        has_manual_input = (self.hover['x'] != 0.0 or
                           self.hover['y'] != 0.0 or
                           self.hover['yaw'] != 0.0)

        # Use manual input if present, otherwise use autonomous commands
        if has_manual_input:
            # Manual control takes priority
            x, y, yaw = self.hover['x'], self.hover['y'], self.hover['yaw']
        else:
            # No manual input - check for autonomous commands
            auto_cmd = self.autonomous_navigator.get_navigation_command()
            x, y, yaw = auto_cmd['x'], auto_cmd['y'], auto_cmd['yaw']

        self.cf.commander.send_hover_setpoint(x, y, yaw, self.hover['height'])

    def updateHover(self, k, v):
        if k == 'autonomous_toggle':
            # Toggle autonomous mode
            new_state = self.autonomous_navigator.set_enabled(not self.autonomous_navigator.enabled)
            # Update window title to reflect mode
            mode_text = "Autonomous Mode" if new_state else "Manual Mode"
            self.setWindowTitle(f'Multi-ranger point cloud - {mode_text} (Press M to toggle)')
        elif (k != 'height'):
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        lmeas = LogConfig(name='Meas', period_in_ms=100)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')

        try:
            self.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.last_pos = position  # Update tracked position
        self.canvas.set_position(position)

    def meas_data(self, timestamp, data, logconf):
        measurement = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right']
        }
        # Update autonomous navigator with current sensor data (not accumulated pointcloud)
        self.autonomous_navigator.update_current_sensors(measurement, self.last_pos)
        self.canvas.set_measurement(measurement)

    def closeEvent(self, event):
        # Stop autonomous navigation and cleanup threads
        if hasattr(self, 'autonomous_navigator'):
            self.autonomous_navigator.set_enabled(False)

        if (self.cf is not None):
            self.cf.close_link()


class Canvas(scene.SceneCanvas):
    def __init__(self, keyupdateCB, autonomous_navigator):
        scene.SceneCanvas.__init__(self, keys=None)
        self.size = 800, 600
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(
            fov=10.0, distance=30.0, up='+z', center=(0.0, 0.0, 0.0))
        self.last_pos = [0, 0, 0]
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.target_marker = visuals.Markers()
        self.pos_data = np.array([0, 0, 0], ndmin=2)
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.target_data = np.array([2, 0, 0], ndmin=2)  # Initial target 2m forward
        self.lines = []
        self.autonomous_navigator = autonomous_navigator

        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)
        self.view.add(self.target_marker)
        for i in range(6):
            line = visuals.Line()
            self.lines.append(line)
            self.view.add(line)

        # Initialize target marker with green color
        self.target_marker.set_data(self.target_data, face_color='green', size=10, edge_color='darkgreen')

        self.keyCB = keyupdateCB

        self.freeze()

        scene.visuals.XYZAxis(parent=self.view.scene)




    def on_key_press(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 1)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', -1)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 1)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', -1)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', -70)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 70)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', -200)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 200)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0.1)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', -0.1)
            if (event.native.key() == QtCore.Qt.Key_M):
                self.keyCB('autonomous_toggle', 1)

    def on_key_release(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_M):
                self.keyCB('autonomous_toggle', 0)

    def set_position(self, pos):
        self.last_pos = pos
        if (PLOT_CF):
            self.pos_data = np.append(self.pos_data, [pos], axis=0)
            self.pos_markers.set_data(self.pos_data, face_color='red', size=5)

        # Update target position based on mode
        if self.autonomous_navigator.target_world_position is None:
            # Manual mode: target follows drone position
            self.update_target_position(pos)
        else:
            # Autonomous mode: use fixed target
            self.update_target_position(pos)  # This will use the fixed target

    def update_target_position(self, drone_pos):
        """Update target marker using fixed target or 2m forward from current position."""
        # Use fixed target from autonomous navigator if available
        if (self.autonomous_navigator.target_world_position is not None):
            target_x, target_y, target_z = self.autonomous_navigator.target_world_position
        else:
            # Default behavior: 2m forward from current drone position
            target_x = drone_pos[0] + 2.0  # 2m forward (positive X)
            target_y = drone_pos[1]        # Same Y as drone
            target_z = drone_pos[2]        # Same Z as drone

        self.target_data = np.array([[target_x, target_y, target_z]])
        self.target_marker.set_data(self.target_data, face_color='green', size=10, edge_color='darkgreen')


    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(math.radians(roll))
        cosp = math.cos(math.radians(pitch))
        cosy = math.cos(math.radians(yaw))

        sinr = math.sin(math.radians(roll))
        sinp = math.sin(math.radians(pitch))
        siny = math.sin(math.radians(yaw))

        roty = np.array([[cosy, -siny, 0],
                         [siny, cosy, 0],
                         [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                         [0, 1, 0],
                         [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                         [0, cosr, -sinr],
                         [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)

    def rotate_and_create_points(self, m):
        data = []
        o = self.last_pos
        roll = m['roll']
        pitch = -m['pitch']
        yaw = m['yaw']

        # Helper function to check distance and add point
        def add_point_if_near(point):
            # Calculate distance from drone to obstacle point
            dx = point[0] - o[0]
            dy = point[1] - o[1]
            distance = math.sqrt(dx * dx + dy * dy)

            # Exclude points greater than 2 meters
            if distance > 2.0:
                return False

            rotated_point = self.rot(roll, pitch, yaw, o, point)
            data.append(rotated_point)
            return True

        if (m['up'] < SENSOR_TH):
            up = [o[0], o[1], o[2] + m['up'] / 1000.0]
            add_point_if_near(up)

        if (m['down'] < SENSOR_TH and PLOT_SENSOR_DOWN):
            down = [o[0], o[1], o[2] - m['down'] / 1000.0]
            add_point_if_near(down)

        if (m['left'] < SENSOR_TH):
            left = [o[0], o[1] + m['left'] / 1000.0, o[2]]
            add_point_if_near(left)

        if (m['right'] < SENSOR_TH):
            right = [o[0], o[1] - m['right'] / 1000.0, o[2]]
            add_point_if_near(right)

        if (m['front'] < SENSOR_TH):
            front = [o[0] + m['front'] / 1000.0, o[1], o[2]]
            add_point_if_near(front)

        if (m['back'] < SENSOR_TH):
            back = [o[0] - m['back'] / 1000.0, o[1], o[2]]
            add_point_if_near(back)

        return data

    def set_measurement(self, measurements):
        data = self.rotate_and_create_points(measurements)
        o = self.last_pos
        for i in range(6):
            if (i < len(data)):
                o = self.last_pos
                self.lines[i].set_data(np.array([o, data[i]]))
            else:
                self.lines[i].set_data(np.array([o, o]))

        if (len(data) > 0):
            self.meas_data = np.append(self.meas_data, data, axis=0)
        self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)


if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    win = MainWindow(URI)
    win.show()
    appQt.exec_()
