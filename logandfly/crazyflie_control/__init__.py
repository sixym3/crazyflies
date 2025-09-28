"""
Modular Crazyflie Control System

A refactored, modular approach to Crazyflie autonomous navigation
with configurable visualization options.
"""

__version__ = "1.0.0"
__author__ = "Crazyflie Control Team"

# Re-export main components for easy importing
from .core.drone_controller import DroneController
from .core.autonomous_navigator import AutonomousNavigator
from .core.occupancy_grid import OccupancyGrid