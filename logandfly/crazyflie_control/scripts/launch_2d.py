#!/usr/bin/env python3
"""
Launch Crazyflie with Matplotlib 2D Visualization

2D grid visualization showing obstacle grid, drone position, target, and planned path.
Perfect for debugging autonomous navigation and path planning.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

import drone_bringup


def main_2d():
    """Launch with 2D visualization."""
    # Override sys.argv to force 2D mode
    original_argv = sys.argv.copy()
    sys.argv = [sys.argv[0], '--2d'] + sys.argv[1:]

    try:
        return drone_bringup.main()
    finally:
        sys.argv = original_argv


if __name__ == '__main__':
    print("Launching Crazyflie with 2D Visualization...")
    print("Perfect for debugging autonomous navigation!")
    sys.exit(main_2d())