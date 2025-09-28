#!/usr/bin/env python3
"""
Test script for persistent obstacle mapping
"""

import sys
from pathlib import Path
import numpy as np
import time

# Add package root to path
package_root = Path(__file__).parent
sys.path.insert(0, str(package_root))

from config import *
from core.occupancy_grid import OccupancyGrid

try:
    from visualization.matplotlib_2d import Matplotlib2DVisualizer, MATPLOTLIB_AVAILABLE
    print(f"Matplotlib available: {MATPLOTLIB_AVAILABLE}")
except ImportError as e:
    print(f"Import error: {e}")
    MATPLOTLIB_AVAILABLE = False

def test_persistent_obstacles():
    """Test persistent obstacle mapping as drone moves around."""

    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available, skipping test")
        return False

    print("Testing Persistent Obstacle Mapping...")
    print("This test simulates a drone moving around and discovering obstacles")

    # Create test grid
    grid = OccupancyGrid(size=GRID_SIZE, resolution=GRID_RESOLUTION)

    # Create visualizer
    try:
        visualizer = Matplotlib2DVisualizer(drone_controller=None)
        print("2D visualizer created successfully")
    except Exception as e:
        print(f"Failed to create visualizer: {e}")
        return False

    # Simulate drone movement and obstacle discovery
    # The drone will move around and "discover" different obstacles at different times

    print("\nSimulating drone movement and obstacle discovery:")

    # Scenario 1: Drone starts at (0,0) and sees obstacles ahead
    print("1. Drone at (0,0) discovers obstacles ahead...")
    drone_pos = (0.0, 0.0)
    obstacles_1 = [(1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]  # Wall ahead

    grid.add_obstacle_points(obstacles_1, drone_pos)
    print(f"   Added {len(obstacles_1)} obstacles. Total persistent: {grid.get_persistent_obstacles_count()}")

    viz_data = {
        'grid': grid,
        'drone_pos': grid.world_to_grid(drone_pos[0], drone_pos[1], drone_pos[0], drone_pos[1]),
        'target_pos': grid.world_to_grid(3.0, 0.0, drone_pos[0], drone_pos[1]),
        'path': [],
        'obstacles_detected': len(obstacles_1)
    }
    visualizer.update(viz_data)
    time.sleep(1.5)

    # Scenario 2: Drone moves right and discovers more obstacles
    print("2. Drone moves to (0.5, 1.0) and discovers side obstacles...")
    drone_pos = (0.5, 1.0)
    obstacles_2 = [(0.0, 2.0), (0.5, 2.0), (1.0, 2.0)]  # Wall to the north

    grid.add_obstacle_points(obstacles_2, drone_pos)
    print(f"   Added {len(obstacles_2)} new obstacles. Total persistent: {grid.get_persistent_obstacles_count()}")

    viz_data = {
        'grid': grid,
        'drone_pos': grid.world_to_grid(drone_pos[0], drone_pos[1], drone_pos[0], drone_pos[1]),
        'target_pos': grid.world_to_grid(3.0, 0.0, drone_pos[0], drone_pos[1]),
        'path': [],
        'obstacles_detected': len(obstacles_2)
    }
    visualizer.update(viz_data)
    time.sleep(1.5)

    # Scenario 3: Drone moves to different area, should still see all obstacles
    print("3. Drone moves to (2.0, 1.5), should see both previous obstacle areas...")
    drone_pos = (2.0, 1.5)
    obstacles_3 = [(3.0, 1.0), (3.0, 1.5), (3.0, 2.0)]  # Another wall to the east

    grid.add_obstacle_points(obstacles_3, drone_pos)
    print(f"   Added {len(obstacles_3)} new obstacles. Total persistent: {grid.get_persistent_obstacles_count()}")

    viz_data = {
        'grid': grid,
        'drone_pos': grid.world_to_grid(drone_pos[0], drone_pos[1], drone_pos[0], drone_pos[1]),
        'target_pos': grid.world_to_grid(4.0, 2.0, drone_pos[0], drone_pos[1]),
        'path': [],
        'obstacles_detected': len(obstacles_3)
    }
    visualizer.update(viz_data)
    time.sleep(1.5)

    # Scenario 4: Drone sees no new obstacles but should still show all previous ones
    print("4. Drone moves to (1.0, 0.5), sees no new obstacles but map persists...")
    drone_pos = (1.0, 0.5)
    obstacles_4 = []  # No new obstacles

    grid.add_obstacle_points(obstacles_4, drone_pos)
    print(f"   Added {len(obstacles_4)} new obstacles. Total persistent: {grid.get_persistent_obstacles_count()}")

    viz_data = {
        'grid': grid,
        'drone_pos': grid.world_to_grid(drone_pos[0], drone_pos[1], drone_pos[0], drone_pos[1]),
        'target_pos': grid.world_to_grid(4.0, 2.0, drone_pos[0], drone_pos[1]),
        'path': [],
        'obstacles_detected': len(obstacles_4)
    }
    visualizer.update(viz_data)
    time.sleep(1.5)

    print(f"\nFinal result: {grid.get_persistent_obstacles_count()} total obstacles mapped persistently")
    print("All obstacles should remain visible even when drone moves away from them!")
    print("Close the window when done inspecting...")

    # Wait briefly for visual inspection
    time.sleep(3.0)

    # Clean up
    visualizer.close()
    print("Persistent obstacle mapping test completed successfully")
    return True

if __name__ == '__main__':
    success = test_persistent_obstacles()
    sys.exit(0 if success else 1)