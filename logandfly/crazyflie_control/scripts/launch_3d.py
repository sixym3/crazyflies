#!/usr/bin/env python3
"""
Launch Crazyflie with PyQt5 3D Visualization

Simple launcher script for the 3D visualization experience.
Equivalent to the original multiranger_pointcloud.py but with modular architecture.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

import drone_bringup


def main_3d():
    """Launch with 3D visualization."""
    # Override sys.argv to force 3D mode
    original_argv = sys.argv.copy()
    sys.argv = [sys.argv[0], '--3d'] + sys.argv[1:]

    try:
        return drone_bringup.main()
    finally:
        sys.argv = original_argv


if __name__ == '__main__':
    print("Launching Crazyflie with 3D Visualization...")
    sys.exit(main_3d())