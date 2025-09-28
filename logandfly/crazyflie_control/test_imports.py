#!/usr/bin/env python3
"""
Test script to verify all imports work correctly
"""

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent
sys.path.insert(0, str(package_root))

def test_imports():
    """Test all module imports."""

    print("Testing core module imports...")

    # Test config import
    try:
        import config
        print("✓ config module imported successfully")
    except ImportError as e:
        print(f"✗ config import failed: {e}")
        return False

    # Test core modules
    try:
        from core.drone_controller import DroneController
        print("✓ DroneController imported successfully")
    except ImportError as e:
        print(f"✗ DroneController import failed: {e}")
        return False

    try:
        from core.autonomous_navigator import AutonomousNavigator
        print("✓ AutonomousNavigator imported successfully")
    except ImportError as e:
        print(f"✗ AutonomousNavigator import failed: {e}")
        return False

    try:
        from core.occupancy_grid import OccupancyGrid
        print("✓ OccupancyGrid imported successfully")
    except ImportError as e:
        print(f"✗ OccupancyGrid import failed: {e}")
        return False

    # Test visualization modules
    try:
        from visualization.ascii_grid import ASCIIGridVisualizer
        print("✓ ASCIIGridVisualizer imported successfully")
    except ImportError as e:
        print(f"✗ ASCIIGridVisualizer import failed: {e}")
        return False

    try:
        from visualization.matplotlib_2d import Matplotlib2DVisualizer, MATPLOTLIB_AVAILABLE
        print(f"✓ Matplotlib2DVisualizer imported successfully (available: {MATPLOTLIB_AVAILABLE})")
    except ImportError as e:
        print(f"✗ Matplotlib2DVisualizer import failed: {e}")
        return False

    try:
        from visualization.pyqt5_3d import PyQt5Visualizer, DEPENDENCIES_AVAILABLE
        print(f"✓ PyQt5Visualizer imported successfully (dependencies available: {DEPENDENCIES_AVAILABLE})")
    except ImportError as e:
        print(f"✗ PyQt5Visualizer import failed: {e}")
        # PyQt5 is optional, so don't fail the test
        print("  (PyQt5 is optional)")

    # Test launch scripts can be imported
    try:
        sys.path.insert(0, str(package_root / "scripts"))
        import drone_bringup
        print("✓ drone_bringup script imported successfully")
    except ImportError as e:
        print(f"✗ drone_bringup import failed: {e}")
        return False

    print("\n✓ All critical imports successful!")
    return True

if __name__ == '__main__':
    success = test_imports()
    sys.exit(0 if success else 1)