#!/usr/bin/env python3
"""
Test script for 2D visualization keyboard input
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

class MockDroneController:
    """Mock drone controller for testing keyboard input."""

    def __init__(self):
        self.autonomous_mode = False

    def update_manual_control(self, control_type, value):
        print(f"Manual control: {control_type} = {value}")

    def toggle_autonomous_mode(self):
        self.autonomous_mode = not self.autonomous_mode
        print(f"Autonomous mode: {'ENABLED' if self.autonomous_mode else 'DISABLED'}")
        return self.autonomous_mode

def test_keyboard_input():
    """Test keyboard input handling with 2D visualization."""

    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available, skipping test")
        return False

    print("Testing 2D Visualization Keyboard Input...")
    print("Instructions:")
    print("  Click on the visualization window to activate it")
    print("  Use arrow keys to move")
    print("  Use W/S for height")
    print("  Use A/D for yaw")
    print("  Use M to toggle autonomous mode")
    print("  Use Q to quit")

    # Create test grid
    grid = OccupancyGrid(size=GRID_SIZE, resolution=GRID_RESOLUTION)

    # Add some test obstacles
    for i in range(45, 55):
        for j in range(45, 55):
            if grid.is_valid_cell(i, j):
                grid.grid[j, i] = 1

    # Create mock drone controller
    mock_controller = MockDroneController()

    # Create visualizer with keyboard support
    try:
        visualizer = Matplotlib2DVisualizer(drone_controller=mock_controller)
        print("2D visualizer with keyboard support created successfully")
    except Exception as e:
        print(f"Failed to create visualizer: {e}")
        return False

    # Create test data
    drone_pos = (50, 50)
    target_pos = (60, 50)
    path = [(50, 50), (52, 50), (55, 50), (58, 50), (60, 50)]

    viz_data = {
        'grid': grid,
        'drone_pos': drone_pos,
        'target_pos': target_pos,
        'path': path,
        'obstacles_detected': 1
    }

    # Update visualization
    try:
        visualizer.update(viz_data)
        print("Visualization updated - test keyboard controls now!")
    except Exception as e:
        print(f"Error updating visualization: {e}")
        return False

    # Keep visualization open for testing
    try:
        import matplotlib.pyplot as plt
        print("\nWindow is open - test keyboard controls!")
        print("The visualization will stay open until you close it or press Q...")

        # Event loop to keep window responsive
        while visualizer.is_open():
            plt.pause(0.1)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")

    # Clean up
    visualizer.close()
    print("Keyboard input test completed")
    return True

if __name__ == '__main__':
    success = test_keyboard_input()
    sys.exit(0 if success else 1)