"""
Matplotlib 2D Visualization

Real-time 2D visualization of the occupancy grid showing:
- Obstacle grid (black/white cells)
- Current drone position (blue circle)
- Final target position (red star)
- Planned path (colored line)
- Individual waypoints (numbered circles)
- Grid coordinates and legend
"""

import sys
from pathlib import Path
import numpy as np
import threading
import time

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from config import *

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Polygon
    import matplotlib.colors as mcolors
    MATPLOTLIB_AVAILABLE = True
except ImportError as e:
    print(f"Matplotlib not available: {e}")
    MATPLOTLIB_AVAILABLE = False


if MATPLOTLIB_AVAILABLE:
    class Matplotlib2DVisualizer:
        """Real-time 2D obstacle grid visualization using matplotlib."""

        def __init__(self, drone_controller=None, update_interval=None, grid_size_meters=None, zoom_follow=None):
            """
            Initialize 2D visualizer.

            Args:
                drone_controller: DroneController instance for keyboard input handling
                update_interval: Update interval in milliseconds (uses config default if None)
                grid_size_meters: Size of grid in meters (uses config default if None)
                zoom_follow: Whether to auto-zoom to follow drone (uses config default if None)
            """
            self.update_interval = update_interval or MATPLOTLIB_UPDATE_INTERVAL
            self.grid_size_meters = grid_size_meters or GRID_SIZE
            self.zoom_follow = zoom_follow if zoom_follow is not None else MATPLOTLIB_AUTO_ZOOM
            self.drone_controller = drone_controller

            # Data storage
            self.current_data = None
            self.data_lock = threading.Lock()

            # Visualization state
            self.fig = None
            self.ax = None
            self.grid_image = None
            self.drone_marker = None
            self.target_marker = None
            self.path_line = None
            self.waypoint_markers = []
            self.legend_elements = []

            # Colors and styles
            self.colors = {
                'obstacle': '#2C3E50',      # Dark blue-gray
                'free': '#ECF0F1',          # Light gray
                'drone': '#3498DB',         # Blue
                'target': '#E74C3C',        # Red
                'path': '#F39C12',          # Orange
                'waypoint': '#9B59B6',      # Purple
                'grid_lines': '#BDC3C7'     # Light gray
            }

            self.setup_plot()

        def setup_plot(self):
            """Initialize the matplotlib plot."""
            # Create figure and axis
            plt.ion()  # Interactive mode
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.fig.suptitle('Crazyflie 2D Navigation Visualization', fontsize=14, fontweight='bold')

            # Set up axis
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_xlabel('X Position (meters)', fontsize=12)
            self.ax.set_ylabel('Y Position (meters)', fontsize=12)

            # Initialize empty plot elements
            self._setup_empty_plot()

            # Add legend
            self._create_legend()

            # Set initial view
            self.ax.set_xlim(-self.grid_size_meters/2, self.grid_size_meters/2)
            self.ax.set_ylim(-self.grid_size_meters/2, self.grid_size_meters/2)

            plt.tight_layout()
            plt.show(block=False)

            # Connect keyboard events if drone controller is available
            if self.drone_controller:
                self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
                self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)

        def _setup_empty_plot(self):
            """Setup empty plot elements."""
            # Create empty grid image
            empty_grid = np.zeros((100, 100))
            self.grid_image = self.ax.imshow(
                empty_grid,
                cmap='gray_r',
                origin='lower',
                extent=[-self.grid_size_meters/2, self.grid_size_meters/2,
                       -self.grid_size_meters/2, self.grid_size_meters/2],
                alpha=0.8,
                interpolation='nearest'
            )

            # Create empty markers
            self.drone_marker = Circle((0, 0), 0.15, color=self.colors['drone'],
                                     zorder=10, linewidth=2, fill=True)
            self.ax.add_patch(self.drone_marker)

            self.target_marker = self.ax.scatter([], [], s=200, c=self.colors['target'],
                                               marker='*', zorder=9, edgecolors='black', linewidth=1)

            # Create empty path line
            self.path_line, = self.ax.plot([], [], color=self.colors['path'],
                                         linewidth=3, zorder=8, alpha=0.8)

        def _create_legend(self):
            """Create visualization legend."""
            legend_elements = [
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=self.colors['drone'],
                          markersize=10, label='Drone Position'),
                plt.Line2D([0], [0], marker='*', color='w', markerfacecolor=self.colors['target'],
                          markersize=15, label='Target Position'),
                plt.Line2D([0], [0], color=self.colors['path'], linewidth=3, label='Planned Path'),
                plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=self.colors['waypoint'],
                          markersize=8, label='Waypoints'),
                patches.Patch(color=self.colors['obstacle'], label='Obstacles'),
                patches.Patch(color=self.colors['free'], label='Free Space')
            ]

            self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.0, 1.0))

        def update(self, data):
            """
            Update visualization with new data.

            Args:
                data: Dictionary containing grid, drone_pos, target_pos, path, etc.
            """
            with self.data_lock:
                self.current_data = data.copy()

            # Update the plot
            self._update_plot()

        def _update_plot(self):
            """Update the matplotlib plot with current data."""
            if self.current_data is None:
                return

            try:
                grid = self.current_data['grid']
                drone_pos = self.current_data['drone_pos']
                target_pos = self.current_data['target_pos']
                path = self.current_data.get('path', [])

                # Convert grid coordinates to world coordinates
                drone_world = self._grid_to_world(drone_pos, grid)
                target_world = self._grid_to_world(target_pos, grid)

                # Update obstacle grid
                self._update_grid_display(grid)

                # Update drone position
                self.drone_marker.center = drone_world

                # Update target position
                if target_world is not None:
                    self.target_marker.set_offsets([target_world])

                # Update path
                self._update_path_display(path, grid)

                # Update title with status
                mode = "AUTONOMOUS" if path else "MANUAL"
                obstacles_detected = self.current_data.get('obstacles_detected', 0)
                grid_stats = grid.get_grid_stats() if hasattr(grid, 'get_grid_stats') else {}
                persistent_obstacles = grid_stats.get('persistent_obstacles', 0)

                self.fig.suptitle(
                    f'Crazyflie 2D Navigation - {mode} Mode | '
                    f'Detected: {obstacles_detected} | Total Map: {persistent_obstacles} | '
                    f'Path: {len(path) if path else 0} waypoints',
                    fontsize=12, fontweight='bold'
                )

                # Auto-zoom to follow drone if enabled
                if self.zoom_follow:
                    self._update_view(drone_world)

                # Refresh display
                plt.draw()
                plt.pause(0.001)  # Small pause to allow GUI update

            except Exception as e:
                print(f"Error updating 2D visualization: {e}")

        def _grid_to_world(self, grid_pos, grid):
            """Convert grid coordinates to world coordinates."""
            if grid_pos is None:
                return None

            try:
                # Grid center is at world (0, 0)
                grid_x, grid_y = grid_pos
                resolution = grid.resolution
                center_x, center_y = grid.center_x, grid.center_y

                world_x = (grid_x - center_x) * resolution
                world_y = (grid_y - center_y) * resolution

                return (world_x, world_y)
            except Exception as e:
                print(f"Error converting grid to world coordinates: {e}")
                return (0, 0)

        def _update_grid_display(self, grid):
            """Update the obstacle grid display."""
            try:
                # Get grid data and flip Y axis for proper display
                grid_data = np.flipud(grid.grid)

                # Update the image
                self.grid_image.set_array(grid_data)
                self.grid_image.set_clim(0, 1)

            except Exception as e:
                print(f"Error updating grid display: {e}")

        def _update_path_display(self, path, grid):
            """Update the path and waypoint display."""
            try:
                if not path:
                    # Clear path
                    self.path_line.set_data([], [])
                    self._clear_waypoint_markers()
                    return

                # Convert path to world coordinates
                path_world = []
                for waypoint in path:
                    world_pos = self._grid_to_world(waypoint, grid)
                    if world_pos:
                        path_world.append(world_pos)

                if path_world:
                    # Update path line
                    x_coords = [pos[0] for pos in path_world]
                    y_coords = [pos[1] for pos in path_world]
                    self.path_line.set_data(x_coords, y_coords)

                    # Update waypoint markers
                    self._update_waypoint_markers(path_world)

            except Exception as e:
                print(f"Error updating path display: {e}")

        def _update_waypoint_markers(self, path_world):
            """Update individual waypoint markers."""
            try:
                # Clear existing waypoint markers
                self._clear_waypoint_markers()

                # Add new waypoint markers (show first 10 waypoints)
                for i, (x, y) in enumerate(path_world[:10]):
                    marker = Circle((x, y), 0.08, color=self.colors['waypoint'],
                                  zorder=9, alpha=0.7)
                    self.ax.add_patch(marker)

                    # Add waypoint number
                    text = self.ax.text(x, y, str(i+1), ha='center', va='center',
                                       fontsize=8, fontweight='bold', color='white', zorder=10)

                    self.waypoint_markers.extend([marker, text])

            except Exception as e:
                print(f"Error updating waypoint markers: {e}")

        def _clear_waypoint_markers(self):
            """Clear all waypoint markers."""
            for marker in self.waypoint_markers:
                try:
                    if hasattr(marker, 'remove'):
                        marker.remove()
                except:
                    pass
            self.waypoint_markers.clear()

        def _update_view(self, drone_world, zoom_size=3.0):
            """Update view to follow drone."""
            try:
                if drone_world is None:
                    return

                x, y = drone_world

                # Set view around drone position
                self.ax.set_xlim(x - zoom_size, x + zoom_size)
                self.ax.set_ylim(y - zoom_size, y + zoom_size)

            except Exception as e:
                print(f"Error updating view: {e}")

        def toggle_zoom_follow(self):
            """Toggle auto-zoom to follow drone."""
            self.zoom_follow = not self.zoom_follow
            print(f"Auto-zoom follow: {'ENABLED' if self.zoom_follow else 'DISABLED'}")

        def reset_view(self):
            """Reset view to show full grid."""
            self.ax.set_xlim(-self.grid_size_meters/2, self.grid_size_meters/2)
            self.ax.set_ylim(-self.grid_size_meters/2, self.grid_size_meters/2)
            plt.draw()

        def close(self):
            """Close the visualization."""
            try:
                plt.close(self.fig)
            except:
                pass

        def is_open(self):
            """Check if visualization window is still open."""
            try:
                return plt.fignum_exists(self.fig.number)
            except:
                return False

        def on_key_press(self, event):
            """Handle key press events for real-time control."""
            if not self.drone_controller or not event.key:
                return

            key = event.key.lower()

            # Movement controls (same as 3D visualization)
            if key == 'left':
                self.drone_controller.update_manual_control('y', 1)
                print("Moving left")
            elif key == 'right':
                self.drone_controller.update_manual_control('y', -1)
                print("Moving right")
            elif key == 'up':
                self.drone_controller.update_manual_control('x', 1)
                print("Moving forward")
            elif key == 'down':
                self.drone_controller.update_manual_control('x', -1)
                print("Moving backward")
            elif key == 'w':
                self.drone_controller.update_manual_control('height', 0.1)
                print("Height up")
            elif key == 's':
                self.drone_controller.update_manual_control('height', -0.1)
                print("Height down")
            elif key == 'a':
                self.drone_controller.update_manual_control('yaw', -70)
                print("Yaw left")
            elif key == 'd':
                self.drone_controller.update_manual_control('yaw', 70)
                print("Yaw right")
            elif key == 'm':
                new_state = self.drone_controller.toggle_autonomous_mode()
                mode = "AUTONOMOUS" if new_state else "MANUAL"
                print(f"Switched to {mode} mode")
                # Update window title
                self.fig.suptitle(f'Crazyflie 2D Navigation - {mode} Mode', fontsize=14, fontweight='bold')
                plt.draw()
            elif key == 'q':
                print("Closing 2D visualization...")
                self.close()

        def on_key_release(self, event):
            """Handle key release events to stop movement."""
            if not self.drone_controller or not event.key:
                return

            key = event.key.lower()

            # Stop movement on key release
            if key in ['left', 'right']:
                self.drone_controller.update_manual_control('y', 0)
            elif key in ['up', 'down']:
                self.drone_controller.update_manual_control('x', 0)
            elif key in ['a', 'd']:
                self.drone_controller.update_manual_control('yaw', 0)

else:
    class Matplotlib2DVisualizer:
        """Dummy class when matplotlib is not available."""

        def __init__(self, drone_controller=None, *args, **kwargs):
            raise ImportError("Matplotlib 2D visualization not available - install matplotlib")

        def update(self, data):
            pass

        def close(self):
            pass

        def is_open(self):
            return False