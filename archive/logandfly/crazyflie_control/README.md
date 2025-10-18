# Crazyflie Modular Control System

A refactored, modular approach to Crazyflie autonomous navigation with configurable visualization options.

## Features

- **Modular Architecture**: Clean separation between control, navigation, and visualization
- **Multiple Visualization Options**: 2D matplotlib, PyQt5 3D, ASCII terminal, or headless operation
- **Autonomous Navigation**: A* path planning with obstacle avoidance
- **Manual Control**: Full manual control with keyboard input
- **No GUI Conflicts**: Run headless to avoid PyQt5/terminal output conflicts

## Quick Start

### Launch Options

```bash
# From the crazyflie_control directory:

# 2D visualization (recommended for debugging)
python3 scripts/launch_2d.py

# 3D visualization (like original multiranger_pointcloud.py)
python3 scripts/launch_3d.py

# Headless mode (no GUI, ASCII terminal only)
python3 scripts/launch_headless.py

# Configurable launcher
python3 scripts/drone_bringup.py --help
```

### Manual Controls

- **Arrow Keys**: Move (left/right/forward/back) - click window to activate
- **W/S**: Height up/down
- **A/D**: Yaw left/right
- **M**: Toggle autonomous mode
- **Q**: Quit (or close window in GUI modes)

### Autonomous Mode

Press **M** to enable autonomous navigation:
- Sets target 2m forward from current position
- Uses A* path planning for obstacle avoidance
- Shows planned path in ASCII grid visualization
- Manual controls override autonomous commands instantly

## Architecture

```
crazyflie_control/
├── core/                    # Core functionality
│   ├── drone_controller.py      # Crazyflie communication & control
│   ├── autonomous_navigator.py  # A* path planning & navigation
│   └── occupancy_grid.py        # Grid mapping & obstacle detection
├── visualization/           # Visualization modules
│   ├── matplotlib_2d.py         # 2D matplotlib grid display
│   ├── ascii_grid.py            # Terminal ASCII grid display
│   └── pyqt5_3d.py             # PyQt5 3D pointcloud viewer
├── scripts/                 # Launch scripts
│   ├── drone_bringup.py         # Main configurable launcher
│   ├── launch_2d.py            # Quick 2D visualization launcher
│   ├── launch_3d.py            # Quick 3D visualization launcher
│   └── launch_headless.py      # Quick headless launcher
├── config.py               # Configuration parameters
└── README.md              # This file
```

## Installation

### Dependencies

```bash
# Core dependencies
pip install cflib numpy

# A* path planning
pip install pyastar2d

# 3D visualization (optional)
pip install PyQt5 vispy

# 2D visualization (optional)
pip install matplotlib
```

### Hardware Requirements

- Crazyflie 2.0 or 2.1
- Crazyradio PA
- Flow deck
- Multiranger deck

## Usage Examples

### 1. 2D Visualization (Recommended)

```bash
# From crazyflie_control directory
python3 scripts/launch_2d.py
```

This provides a clear 2D view of the obstacle grid showing:
- **Persistent obstacle map** (dark/light cells) - obstacles remain visible after discovery
- Drone position (blue circle)
- Target position (red star)
- Planned path (orange line)
- Numbered waypoints (purple circles)
- Real-time status: detected obstacles vs. total mapped obstacles

### 2. Standard 3D Visualization

```bash
# From crazyflie_control directory
python3 scripts/launch_3d.py
```

This provides the same experience as the original `multiranger_pointcloud.py` but with clean modular architecture.

### 3. Headless Operation

```bash
# From crazyflie_control directory
python3 scripts/launch_headless.py
```

Perfect for:
- Remote operation via SSH
- Avoiding GUI conflicts
- Automated testing
- Running on headless systems

### 4. Custom Configuration

```bash
# 2D visualization with custom URI
python3 scripts/drone_bringup.py --2d --uri radio://0/80/2M/E7E7E7E7E3

# ASCII visualization only
python3 scripts/drone_bringup.py --ascii-only --uri radio://0/80/2M/E7E7E7E7E3

# Headless with custom URI
python3 scripts/drone_bringup.py --headless --uri radio://0/80/2M/E7E7E7E7E4
```

## Configuration

Edit `config.py` to customize:

- **Connection**: Default URI, connection timeouts
- **Control**: Speed factors, waypoint tolerance
- **Navigation**: Grid resolution, obstacle inflation
- **Visualization**: Update intervals, display options

## Troubleshooting

### PyQt5 Issues

If PyQt5 causes conflicts or crashes:

```bash
# Use headless mode instead
python scripts/launch_headless.py
```

### Path Planning Issues

Enable debug output to see A* planning details:
- ASCII grid shows planned path with `*` and `W` symbols
- Drone position shown as `D`, target as `T`, obstacles as `#`

### Connection Problems

```bash
# Check available Crazyflies
python scripts/drone_bringup.py --uri radio://0/80/2M/E7E7E7E7E7 --headless
```

## Development

### Adding New Visualizations

1. Create new module in `visualization/`
2. Implement callback interface for data updates
3. Add to launcher scripts or `drone_bringup.py`

### Extending Navigation

1. Modify `AutonomousNavigator` in `core/autonomous_navigator.py`
2. Add new path planning algorithms
3. Customize obstacle detection logic

### Custom Launch Scripts

Create custom launchers by importing and configuring the core modules:

```python
from core.drone_controller import DroneController
from core.autonomous_navigator import AutonomousNavigator
from visualization.ascii_grid import ASCIIGridVisualizer

# Your custom configuration here
```

## Comparison with Original

| Feature | Original | Modular |
|---------|----------|---------|
| Architecture | Monolithic | Modular |
| Visualization | PyQt5 3D only | Multiple options |
| GUI Conflicts | Common | Eliminated |
| Headless Mode | No | Yes |
| Extensibility | Difficult | Easy |
| Testing | Coupled | Independent |

## License

Same as original Crazyflie Python Library (GPL v2)