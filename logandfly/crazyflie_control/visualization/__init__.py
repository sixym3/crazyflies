"""
Visualization modules for Crazyflie Control System
"""

from .ascii_grid import ASCIIGridVisualizer

try:
    from .pyqt5_3d import PyQt5Visualizer
    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False
    print("PyQt5 not available, 3D visualization disabled")

try:
    from .matplotlib_2d import Matplotlib2DVisualizer
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Matplotlib not available, 2D visualization disabled")

__all__ = ['ASCIIGridVisualizer']
if PYQT5_AVAILABLE:
    __all__.append('PyQt5Visualizer')
if MATPLOTLIB_AVAILABLE:
    __all__.append('Matplotlib2DVisualizer')