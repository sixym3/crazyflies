#!/usr/bin/env python3
"""
Test script for 2D visualization integration
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

def test_2d_visualization():
    """Test the 2D matplotlib visualization with sample data."""

    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available, skipping test")
        return False

    print("Testing 2D Visualization...")

    # Create test grid
    grid = OccupancyGrid(size=GRID_SIZE, resolution=GRID_RESOLUTION)

    # Add some test obstacles
    print("Adding test obstacles...")
    for i in range(45, 55):
        for j in range(45, 55):
            if grid.is_valid_cell(i, j):
                grid.grid[j, i] = 1  # Mark as obstacle (note: grid[y, x])

    for i in range(30, 40):
        for j in range(60, 70):
            if grid.is_valid_cell(i, j):
                grid.grid[j, i] = 1  # Mark as obstacle (note: grid[y, x])

    # Create visualizer (without drone controller for testing)
    try:
        visualizer = Matplotlib2DVisualizer(drone_controller=None)
        print("2D visualizer created successfully")
    except Exception as e:
        print(f"Failed to create visualizer: {e}")
        return False

    # Test data updates
    print("Testing visualization updates...")

    # Simulate drone movement
    positions = [
        (40, 40),  # Start position
        (42, 42),  # Moving diagonally
        (45, 45),  # Near obstacle
        (48, 45),  # Around obstacle
        (50, 42),  # Past obstacle
        (55, 40),  # Target approach
        (60, 40),  # At target
    ]

    target_pos = (60, 40)  # Fixed target

    for i, drone_pos in enumerate(positions):
        print(f"Update {i+1}: Drone at {drone_pos}")

        # Create sample path (simplified)
        if i < len(positions) - 1:
            path = positions[i:i+3]  # Next few waypoints
        else:
            path = []

        # Create visualization data
        viz_data = {
            'grid': grid,
            'drone_pos': drone_pos,
            'target_pos': target_pos,
            'path': path,
            'obstacles_detected': 2
        }

        # Update visualization
        try:
            visualizer.update(viz_data)
            print(f"  Visualization updated successfully")
            time.sleep(1.0)  # Pause to see the update
        except Exception as e:
            print(f"  Error updating visualization: {e}")
            return False

        # Check if window is still open
        if not visualizer.is_open():
            print("  Visualization window closed")
            break

    print("Test completed! Waiting 3 seconds before closing...")

    # Wait briefly for visual inspection
    time.sleep(3.0)

    # Clean up
    visualizer.close()
    print("2D visualization test completed successfully")
    return True

if __name__ == '__main__':
    success = test_2d_visualization()
    sys.exit(0 if success else 1)