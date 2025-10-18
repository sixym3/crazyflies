"""
Autonomous Navigation System

A* path planning and autonomous navigation for Crazyflie drones.
Handles obstacle detection, path planning, and navigation commands.
"""

import math
import threading
import time
from collections import deque

import numpy as np

try:
    import pyastar2d
except ImportError:
    print("Warning: pyastar2d not installed. Install with: pip install pyastar2d")
    pyastar2d = None

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from core.occupancy_grid import OccupancyGrid
from config import *


class AsyncPathPlanner:
    """Background thread for A* path planning."""

    def __init__(self):
        self.current_path = deque()
        self.planning_lock = threading.Lock()
        self.needs_planning = False  # Flag to trigger planning (protected by lock)
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
            self.needs_planning = True

    def get_current_path(self):
        """Get the current planned path (thread-safe)."""
        with self.planning_lock:
            return list(self.current_path)

    def remove_first_waypoint(self):
        """Remove the first waypoint from the path (thread-safe)."""
        with self.planning_lock:
            if self.current_path:
                removed = self.current_path.popleft()
                return removed, len(self.current_path) > 0
            return None, False

    def has_waypoints(self):
        """Check if there are waypoints in the path (thread-safe)."""
        with self.planning_lock:
            return len(self.current_path) > 0

    def _planning_worker(self):
        """Background worker for path planning."""
        while not self.stop_planning.is_set():
            # Poll for planning requests every 100ms
            should_plan = False
            with self.planning_lock:
                if self.needs_planning:
                    should_plan = True
                    self.needs_planning = False

            if should_plan:
                if pyastar2d is None:
                    continue

                with self.planning_lock:
                    if self.grid is not None and self.target_pos is not None:
                        try:
                            # Prepare grid for A* with distance-based costs
                            weights = np.ones_like(self.grid.grid, dtype=np.float32)

                            # Add distance-based cost to guide A* toward target
                            goal_row, goal_col = self.target_pos[1], self.target_pos[0]
                            for row in range(weights.shape[0]):
                                for col in range(weights.shape[1]):
                                    if self.grid.grid[row, col] == 0:  # Only for free cells
                                        # Calculate distance to goal
                                        distance = np.sqrt((row - goal_row)**2 + (col - goal_col)**2)
                                        # Cost = base_cost + small_distance_penalty
                                        weights[row, col] = 1.0 + distance * 0.01

                            weights[self.grid.grid == 1] = np.inf  # Obstacles = infinity

                            # Plan path from drone position to target
                            start = (self.grid.drone_pos[1], self.grid.drone_pos[0])  # (row, col)
                            goal = (self.target_pos[1], self.target_pos[0])  # (row, col)

                            # Bounds checking
                            if not (0 <= start[0] < weights.shape[0] and 0 <= start[1] < weights.shape[1]):
                                print(f"Start position {start} out of bounds")
                                self.current_path.clear()
                                continue
                            if not (0 <= goal[0] < weights.shape[0] and 0 <= goal[1] < weights.shape[1]):
                                print(f"Goal position {goal} out of bounds")
                                self.current_path.clear()
                                continue

                            # Run A*
                            path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)

                            if path is not None and len(path) > 1:
                                # Convert path back to (x, y) and store
                                self.current_path.clear()
                                for point in path[1:]:  # Skip current position
                                    self.current_path.append((point[1], point[0]))  # Convert (row,col) to (x,y)
                                print(f"Path found with {len(path)-1} waypoints")
                            else:
                                print("No path found to target")
                                self.current_path.clear()

                        except Exception as e:
                            print(f"Path planning error: {e}")
                            self.current_path.clear()
            else:
                # Sleep when no planning needed
                time.sleep(0.1)


class AutonomousNavigator:
    """Main autonomous navigation system."""

    def __init__(self, enable_visualization=False):
        """
        Initialize autonomous navigator.

        Args:
            enable_visualization: Whether to enable debug visualization
        """
        self.enabled = False
        self.occupancy_grid = OccupancyGrid()
        self.path_planner = AsyncPathPlanner()
        self.current_waypoint = None
        self.waypoint_tolerance = WAYPOINT_TOLERANCE
        self.last_drone_position = (0, 0, 0)
        self.target_world_position = None  # Fixed target in world coordinates
        self.current_sensor_data = {}

        # Path checking counter for periodic updates
        self.path_check_counter = 0
        self.path_check_interval = 5  # Check for new paths every 5 navigation calls

        # Visualization
        self.enable_visualization = enable_visualization
        self.visualization_callbacks = []

    def add_visualization_callback(self, callback):
        """Add callback for visualization updates."""
        self.visualization_callbacks.append(callback)

    def set_enabled(self, enabled):
        """Enable or disable autonomous navigation."""
        self.enabled = enabled
        if enabled:
            # Set fixed target 2m forward from current drone position
            current_x, current_y, current_z = self.last_drone_position
            self.target_world_position = (current_x + TARGET_DISTANCE, current_y, current_z)
            print(f"Autonomous mode: ENABLED - Target set to {self.target_world_position}")
            print(f"DEBUG: Current drone position: {self.last_drone_position}")

            self.path_planner.start()

            # Immediately request first path with current grid state
            # This ensures path planning starts right away instead of waiting for next sensor update
            self._do_autonomous_planning()
            print("DEBUG: Initial path request sent")
        else:
            self.path_planner.stop()
            self.current_waypoint = None
            self.target_world_position = None
            print("Autonomous mode: DISABLED")

        return self.enabled

    def update_position(self, position):
        """Update drone position."""
        self.last_drone_position = position

    def update_sensors(self, sensor_data, drone_position):
        """Update with current sensor readings."""
        self.last_drone_position = drone_position
        self.current_sensor_data = sensor_data

        # Always update grid tracking
        self._update_grid_tracking(sensor_data, drone_position)

        # Only do autonomous planning if enabled
        if self.enabled:
            self._do_autonomous_planning()

    def _update_grid_tracking(self, sensor_data, drone_position):
        """Update occupancy grid with current obstacles."""
        # Create obstacles from current sensor readings
        current_obstacles = self._create_obstacles_from_sensors(sensor_data, drone_position)

        # Update occupancy grid
        nearby_count, total_count = self.occupancy_grid.add_obstacle_points(
            current_obstacles, drone_position)

        # Update visualization callbacks
        for callback in self.visualization_callbacks:
            callback({
                'grid': self.occupancy_grid,
                'drone_pos': self.occupancy_grid.drone_pos,
                'target_pos': self._get_target_grid_pos(),
                'path': self.path_planner.get_current_path() if self.enabled else None,
                'obstacles_detected': len(current_obstacles)
            })

    def _create_obstacles_from_sensors(self, sensor_data, drone_position):
        """Create obstacle points from multiranger sensor readings."""
        obstacles = []
        drone_x, drone_y, drone_z = drone_position

        # Get sensor readings in meters
        front_dist = sensor_data.get('front', 4000) / 1000.0
        back_dist = sensor_data.get('back', 4000) / 1000.0
        left_dist = sensor_data.get('left', 4000) / 1000.0
        right_dist = sensor_data.get('right', 4000) / 1000.0

        obstacle_threshold = OBSTACLE_THRESHOLD_M

        # Create obstacles with depth for better representation
        if front_dist < obstacle_threshold:
            base_pos = [drone_x + front_dist, drone_y, drone_z]
            obstacles.append(base_pos)
            # Add depth
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x + front_dist + offset, drone_y, drone_z])

        if back_dist < obstacle_threshold:
            base_pos = [drone_x - back_dist, drone_y, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x - back_dist - offset, drone_y, drone_z])

        if left_dist < obstacle_threshold:
            base_pos = [drone_x, drone_y + left_dist, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x, drone_y + left_dist + offset, drone_z])

        if right_dist < obstacle_threshold:
            base_pos = [drone_x, drone_y - right_dist, drone_z]
            obstacles.append(base_pos)
            for offset in [0.1, 0.2]:
                obstacles.append([drone_x, drone_y - right_dist - offset, drone_z])

        return obstacles

    def _do_autonomous_planning(self):
        """Perform autonomous path planning."""
        if self.target_world_position is None:
            print("DEBUG: Planning skipped - no target set")
            return

        # Convert fixed world target to grid coordinates
        target_x, target_y = self.target_world_position[0], self.target_world_position[1]
        drone_x, drone_y = self.last_drone_position[0], self.last_drone_position[1]
        target_grid_pos = self.occupancy_grid.world_to_grid(target_x, target_y, drone_x, drone_y)

        grid_stats = self.occupancy_grid.get_grid_stats()
        print(f"DEBUG: Requesting path - Drone: ({drone_x:.2f}, {drone_y:.2f}) -> Target: ({target_x:.2f}, {target_y:.2f}), Grid target: {target_grid_pos}, Obstacles: {grid_stats['obstacles']}")

        self.path_planner.request_path(self.occupancy_grid, target_grid_pos)

    def _get_target_grid_pos(self):
        """Get target position in grid coordinates."""
        if self.target_world_position is not None:
            target_x, target_y = self.target_world_position[0], self.target_world_position[1]
            drone_x, drone_y = self.last_drone_position[0], self.last_drone_position[1]
            return self.occupancy_grid.world_to_grid(target_x, target_y, drone_x, drone_y)
        else:
            return self.occupancy_grid.get_target_position()

    def get_navigation_command(self):
        """Generate navigation command using planned path."""
        if not self.enabled:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Increment counter and periodically check for new paths
        self.path_check_counter += 1
        should_refresh_path = (self.path_check_counter % self.path_check_interval == 0)

        # Check if we need to get a new waypoint or refresh periodically
        if self.current_waypoint is None or should_refresh_path:
            has_waypoints = self.path_planner.has_waypoints()
            if has_waypoints:
                # Get the first waypoint from the path
                path = self.path_planner.get_current_path()
                if path:
                    # Only update waypoint if we don't have one or if it's different
                    new_waypoint = path[0]
                    if self.current_waypoint is None or self.current_waypoint != new_waypoint:
                        self.current_waypoint = new_waypoint
                        print(f"DEBUG: New waypoint set to {self.current_waypoint}")
                        if should_refresh_path:
                            print(f"Path refreshed: new waypoint {self.current_waypoint}")
            elif self.path_check_counter % 20 == 0:  # Log every 2 seconds
                print(f"DEBUG: No waypoints available in path")

        if self.current_waypoint is None:
            if self.path_check_counter % 20 == 0:  # Log every 2 seconds
                print("DEBUG: No current waypoint - returning zero command")
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

        # Check if we've reached the current waypoint
        if distance < self.waypoint_tolerance:
            # Remove the reached waypoint from the actual path
            removed_waypoint, has_more = self.path_planner.remove_first_waypoint()

            if has_more:
                # Get the next waypoint
                path = self.path_planner.get_current_path()
                self.current_waypoint = path[0] if path else None
            else:
                # No more waypoints, stop
                self.current_waypoint = None
                print("Target reached - all waypoints completed")
                return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Generate movement command
        if distance > 0:
            cmd_x = (dx / distance) * AUTO_SPEED_FACTOR
            cmd_y = (dy / distance) * AUTO_SPEED_FACTOR

            # Limit maximum speed
            max_speed = AUTO_SPEED_FACTOR
            if abs(cmd_x) > max_speed:
                cmd_x = max_speed if cmd_x > 0 else -max_speed
            if abs(cmd_y) > max_speed:
                cmd_y = max_speed if cmd_y > 0 else -max_speed

            if self.path_check_counter % 20 == 0:  # Log every 2 seconds
                print(f"DEBUG: Command - waypoint: {waypoint_world}, dist: {distance:.2f}m, cmd: ({cmd_x:.2f}, {cmd_y:.2f})")

            return {'x': cmd_x, 'y': cmd_y, 'yaw': 0.0}
        else:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def get_status(self):
        """Get navigation status."""
        path = self.path_planner.get_current_path()
        return {
            'enabled': self.enabled,
            'target_world': self.target_world_position,
            'current_waypoint': self.current_waypoint,
            'path_length': len(path) if path else 0,
            'grid_stats': self.occupancy_grid.get_grid_stats()
        }