#!/usr/bin/env python3
"""
Launch Crazyflie in Headless Mode

No GUI interface - terminal-based control with ASCII grid visualization.
Perfect for remote operation, testing, or when GUI conflicts occur.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

import drone_bringup


def main_headless():
    """Launch in headless mode."""
    # Override sys.argv to force headless mode
    original_argv = sys.argv.copy()
    sys.argv = [sys.argv[0], '--headless'] + sys.argv[1:]

    try:
        return drone_bringup.main()
    finally:
        sys.argv = original_argv


if __name__ == '__main__':
    print("Launching Crazyflie in Headless Mode...")
    print("No GUI - Terminal control with ASCII visualization")
    sys.exit(main_headless())