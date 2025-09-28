"""
Configuration settings for Crazyflie Control System
"""

# Crazyflie connection
DEFAULT_URI = 'radio://0/80/2M/E7E7E7E7E2'

# Control parameters
SPEED_FACTOR = 0.3
AUTO_SPEED_FACTOR = 0.15
AUTO_TURN_RATE = 20  # degrees/second

# Sensor parameters
SENSOR_THRESHOLD_MM = 2000  # mm
OBSTACLE_THRESHOLD_M = 2.0  # meters

# Grid parameters
GRID_RESOLUTION = 0.1  # meters per cell
GRID_SIZE = 10.0  # grid size in meters (10m x 10m)
OBSTACLE_INFLATION = 1  # cells to inflate obstacles

# Navigation parameters
WAYPOINT_TOLERANCE = 0.2  # meters
TARGET_DISTANCE = 2.0  # meters forward for autonomous target

# Visualization parameters
ENABLE_ASCII_GRID = True
ASCII_UPDATE_INTERVAL = 10  # updates between ASCII prints
VIEW_SIZE = 20  # cells in each direction for ASCII view

# 3D Visualization parameters
PLOT_CRAZYFLIE = True
PLOT_SENSOR_DOWN = True
CANVAS_SIZE = (800, 600)
CAMERA_DISTANCE = 30.0

# 2D Visualization parameters
MATPLOTLIB_UPDATE_INTERVAL = 200  # ms
MATPLOTLIB_FIGURE_SIZE = (10, 10)
MATPLOTLIB_AUTO_ZOOM = True
MATPLOTLIB_ZOOM_SIZE = 3.0  # meters around drone
MATPLOTLIB_GRID_ALPHA = 0.8

# Timing
HOVER_COMMAND_INTERVAL = 100  # ms
LOG_PERIOD = 100  # ms