#!/usr/bin/env python3
"""
Crazyflie Drone Bringup Script

Configurable launch script for Crazyflie control with various visualization options.
Supports manual control, autonomous navigation, and multiple visualization modes.

Usage:
    python drone_bringup.py [options]

Examples:
    python drone_bringup.py --headless              # No GUI, ASCII only
    python drone_bringup.py --3d                    # PyQt5 3D visualization
    python drone_bringup.py --ascii-only            # Terminal ASCII grid only
    python drone_bringup.py --uri radio://0/80/2M/E7E7E7E7E3  # Custom URI
"""

import argparse
import sys
import time
import threading
from pathlib import Path

# Add parent directory to path for imports
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

# Import modules directly
from core.drone_controller import DroneController
from core.autonomous_navigator import AutonomousNavigator
from visualization.ascii_grid import ASCIIGridVisualizer
from config import *

try:
    from visualization.pyqt5_3d import PyQt5Visualizer, DEPENDENCIES_AVAILABLE as PYQT5_AVAILABLE
except ImportError:
    PYQT5_AVAILABLE = False

try:
    from visualization.matplotlib_2d import Matplotlib2DVisualizer, MATPLOTLIB_AVAILABLE
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class DroneControlSystem:
    """Main drone control system with configurable visualization."""

    def __init__(self, args):
        """Initialize drone control system based on arguments."""
        self.args = args
        self.running = True

        # Create autonomous navigator
        self.navigator = AutonomousNavigator(enable_visualization=True)

        # Create drone controller
        self.controller = DroneController(
            uri=args.uri,
            autonomous_navigator=self.navigator
        )

        # Setup visualizations
        self.visualizers = []
        self._setup_visualizations()

        # Control timer
        self.control_timer = None

    def _setup_visualizations(self):
        """Setup visualization based on command line arguments."""
        # ASCII Grid Visualization
        if self.args.ascii or self.args.headless or self.args.ascii_only:
            ascii_viz = ASCIIGridVisualizer(enabled=True)
            self.navigator.add_visualization_callback(ascii_viz.update)
            self.visualizers.append(ascii_viz)
            print("ASCII grid visualization enabled")

        # Matplotlib 2D Visualization
        if self.args.gui_2d and MATPLOTLIB_AVAILABLE:
            try:
                matplotlib_viz = Matplotlib2DVisualizer(drone_controller=self.controller)
                self.navigator.add_visualization_callback(matplotlib_viz.update)
                self.visualizers.append(matplotlib_viz)
                print("Matplotlib 2D visualization enabled")
            except Exception as e:
                print(f"Failed to setup 2D visualization: {e}")
                if not self.args.ascii:
                    # Fallback to ASCII if 2D fails
                    ascii_viz = ASCIIGridVisualizer(enabled=True)
                    self.navigator.add_visualization_callback(ascii_viz.update)
                    self.visualizers.append(ascii_viz)
                    print("Falling back to ASCII visualization")

        elif self.args.gui_2d and not MATPLOTLIB_AVAILABLE:
            print("Matplotlib 2D visualization requested but not available")
            print("Install matplotlib: pip install matplotlib")
            # Fallback to ASCII
            ascii_viz = ASCIIGridVisualizer(enabled=True)
            self.navigator.add_visualization_callback(ascii_viz.update)
            self.visualizers.append(ascii_viz)
            print("Using ASCII visualization instead")

        # PyQt5 3D Visualization
        if self.args.gui_3d and PYQT5_AVAILABLE:
            try:
                pyqt5_viz = PyQt5Visualizer(self.controller)
                self.visualizers.append(pyqt5_viz)
                print("PyQt5 3D visualization enabled")
            except Exception as e:
                print(f"Failed to setup PyQt5 visualization: {e}")
                if not self.args.ascii and not self.args.gui_2d:
                    # Fallback to ASCII if 3D fails and no 2D
                    ascii_viz = ASCIIGridVisualizer(enabled=True)
                    self.navigator.add_visualization_callback(ascii_viz.update)
                    self.visualizers.append(ascii_viz)
                    print("Falling back to ASCII visualization")

        elif self.args.gui_3d and not PYQT5_AVAILABLE:
            print("PyQt5 3D visualization requested but not available")
            print("Install PyQt5 and vispy: pip install PyQt5 vispy")
            # Fallback to ASCII
            ascii_viz = ASCIIGridVisualizer(enabled=True)
            self.navigator.add_visualization_callback(ascii_viz.update)
            self.visualizers.append(ascii_viz)
            print("Using ASCII visualization instead")

    def start(self):
        """Start the drone control system."""
        print("Starting Crazyflie Control System...")
        print(f"URI: {self.controller.uri}")
        print(f"Visualizations: {len(self.visualizers)} active")

        # Connect to drone
        self.controller.connect()

        # Start control timer
        self._start_control_timer()

        # Handle different run modes
        if self.args.gui_3d and any(hasattr(viz, 'run') for viz in self.visualizers):
            # Run PyQt5 GUI (blocking)
            for viz in self.visualizers:
                if hasattr(viz, 'run'):
                    print("Starting 3D visualization (press M for autonomous mode)...")
                    return viz.run()
        elif self.args.gui_2d:
            # Run with 2D visualization (non-blocking)
            print("Starting 2D visualization (press M for autonomous mode)...")
            self._run_with_2d_visualization()
        else:
            # Run headless or ASCII-only mode
            self._run_headless()

    def _start_control_timer(self):
        """Start the control command timer."""
        self.control_timer = threading.Timer(HOVER_COMMAND_INTERVAL / 1000.0, self._send_control_commands)
        self.control_timer.daemon = True
        self.control_timer.start()

    def _send_control_commands(self):
        """Send control commands periodically."""
        if self.running:
            self.controller.send_control_command()
            self._start_control_timer()

    def _run_headless(self):
        """Run in headless mode with keyboard input."""
        print("\\nCrazyflie Control System Running")
        print("Controls:")
        print("  Arrow keys: Move (left/right/forward/back)")
        print("  W/S: Height up/down")
        print("  A/D: Yaw left/right")
        print("  M: Toggle autonomous mode")
        print("  Q: Quit")
        print("\\nPress Enter after each command...")

        try:
            while self.running:
                cmd = input().strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 'm':
                    new_state = self.controller.toggle_autonomous_mode()
                    mode = "AUTONOMOUS" if new_state else "MANUAL"
                    print(f"Switched to {mode} mode")
                elif cmd == 'w':
                    self.controller.update_manual_control('height', 0.1)
                    print("Height up")
                elif cmd == 's':
                    self.controller.update_manual_control('height', -0.1)
                    print("Height down")
                elif cmd == 'a':
                    self.controller.update_manual_control('yaw', -70)
                    time.sleep(0.1)
                    self.controller.update_manual_control('yaw', 0)
                    print("Yaw left")
                elif cmd == 'd':
                    self.controller.update_manual_control('yaw', 70)
                    time.sleep(0.1)
                    self.controller.update_manual_control('yaw', 0)
                    print("Yaw right")
                elif cmd == 'status':
                    status = self.controller.get_status()
                    nav_status = self.navigator.get_status()
                    print(f"\\nStatus:")
                    print(f"  Connected: {status['connected']}")
                    print(f"  Position: {status['position']}")
                    print(f"  Autonomous: {status['autonomous_enabled']}")
                    if nav_status['enabled']:
                        print(f"  Target: {nav_status['target_world']}")
                        print(f"  Path length: {nav_status['path_length']}")
                        print(f"  Grid obstacles: {nav_status['grid_stats']['obstacles']}")
                elif cmd == 'help':
                    print("\\nAvailable commands:")
                    print("  w/s - height up/down")
                    print("  a/d - yaw left/right")
                    print("  m - toggle autonomous mode")
                    print("  status - show system status")
                    print("  q - quit")
                elif cmd == '':
                    continue
                else:
                    print(f"Unknown command: {cmd} (type 'help' for commands)")

        except KeyboardInterrupt:
            print("\\nShutdown requested...")

        self.stop()

    def _run_with_2d_visualization(self):
        """Run with 2D matplotlib visualization."""
        print("\\nCrazyflie Control System Running with 2D Visualization")
        print("Controls:")
        print("  Arrow keys: Move (left/right/forward/back)")
        print("  W/S: Height up/down")
        print("  A/D: Yaw left/right")
        print("  M: Toggle autonomous mode")
        print("  Q: Quit")
        print("\\nClick on the 2D visualization window to activate keyboard controls...")

        try:
            # Import matplotlib for event handling
            import matplotlib.pyplot as plt

            # Keep checking if visualization is open and handle matplotlib events
            while self.running:
                # Check if 2D visualization window is still open
                viz_open = any(viz.is_open() for viz in self.visualizers
                              if hasattr(viz, 'is_open'))

                if not viz_open:
                    print("\\n2D visualization window closed, shutting down...")
                    break

                # Process matplotlib events (keyboard input, window events, etc.)
                plt.pause(0.1)  # Process events and refresh display

        except KeyboardInterrupt:
            print("\\nShutdown requested...")

        self.stop()

    def stop(self):
        """Stop the drone control system."""
        print("Stopping drone control system...")
        self.running = False

        if self.control_timer:
            self.control_timer.cancel()

        # Stop autonomous navigation
        if self.navigator:
            self.navigator.set_enabled(False)

        # Disconnect drone
        self.controller.disconnect()

        # Close visualizations
        for viz in self.visualizers:
            if hasattr(viz, 'close'):
                viz.close()

        print("Shutdown complete")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Crazyflie Drone Control with configurable visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --headless              # No GUI, ASCII terminal only
  %(prog)s --2d                    # Matplotlib 2D visualization
  %(prog)s --3d                    # PyQt5 3D visualization
  %(prog)s --ascii-only            # ASCII grid only, no GUI
  %(prog)s --uri radio://0/80/2M/E7E7E7E7E3  # Custom Crazyflie URI
        """
    )

    # Connection options
    parser.add_argument(
        '--uri',
        default=DEFAULT_URI,
        help=f'Crazyflie URI (default: {DEFAULT_URI})'
    )

    # Visualization options (mutually exclusive group)
    viz_group = parser.add_mutually_exclusive_group()
    viz_group.add_argument(
        '--headless',
        action='store_true',
        help='Run without GUI, ASCII terminal output only'
    )
    viz_group.add_argument(
        '--2d',
        dest='gui_2d',
        action='store_true',
        help='Run with Matplotlib 2D visualization'
    )
    viz_group.add_argument(
        '--3d',
        dest='gui_3d',
        action='store_true',
        help='Run with PyQt5 3D visualization'
    )
    viz_group.add_argument(
        '--ascii-only',
        action='store_true',
        help='ASCII grid visualization only, no GUI'
    )

    # Additional options
    parser.add_argument(
        '--ascii',
        action='store_true',
        help='Enable ASCII grid visualization (in addition to other viz)'
    )

    args = parser.parse_args()

    # Set defaults if no visualization specified
    if not any([args.headless, args.gui_2d, args.gui_3d, args.ascii_only]):
        # Default to 2D if available, then 3D, otherwise headless
        if MATPLOTLIB_AVAILABLE:
            args.gui_2d = True
        elif PYQT5_AVAILABLE:
            args.gui_3d = True
        else:
            args.headless = True

    return args


def main():
    """Main entry point."""
    args = parse_arguments()

    print("Crazyflie Modular Control System")
    print("================================")

    # Create and start control system
    control_system = DroneControlSystem(args)

    try:
        exit_code = control_system.start()
        return exit_code or 0
    except Exception as e:
        print(f"Error: {e}")
        control_system.stop()
        return 1


if __name__ == '__main__':
    sys.exit(main())