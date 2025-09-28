#!/usr/bin/env python3
"""
Live path plotter for Crazyflie flights.
Provides real-time visualization of the drone's path during flight execution.
"""

import threading
import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Set backend before importing pyplot
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle


class LivePathPlotter:
    """
    Real-time path plotter that visualizes drone trajectory during flight.
    Runs in a separate thread to avoid blocking flight operations.
    """

    def __init__(self, logs_array, log_count_ref, update_interval=200):
        """
        Initialize the live plotter.

        Args:
            logs_array: Reference to the shared numpy logs array
            log_count_ref: Reference to the log_count variable (list with single element)
            update_interval: Update interval in milliseconds (default 200ms = 5Hz)
        """
        self.logs = logs_array
        self.log_count_ref = log_count_ref  # Should be a list with one element for reference
        self.update_interval = update_interval
        self.running = False
        self.thread = None

        # Initialize plot elements
        self.fig = None
        self.ax = None
        self.line = None
        self.start_marker = None
        self.end_marker = None
        self.current_marker = None
        self.stats_text = None
        self.ani = None

        # Data tracking
        self.last_update_count = 0
        self.path_length = 0.0

    def start(self):
        """Start the live plotting in a separate thread."""
        if self.running:
            return

        self.running = True
        self.thread = threading.Thread(target=self._run_plotter, daemon=True)
        self.thread.start()
        print("Live plotter started...")

    def stop(self):
        """Stop the live plotting."""
        self.running = False
        if self.ani:
            self.ani.event_source.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.fig:
            plt.close(self.fig)
        print("Live plotter stopped.")

    def _run_plotter(self):
        """Main plotting thread function."""
        try:
            # Set up matplotlib for non-blocking operation
            plt.ion()

            # Create figure and axis
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.ax.set_title("Live Crazyflie Path (Bird's-eye: X vs Y)")
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            self.ax.set_aspect("equal", adjustable="box")
            self.ax.grid(True)

            # Initialize plot elements
            self.line, = self.ax.plot([], [], 'b-', linewidth=1.5, label='Path')
            self.start_marker, = self.ax.plot([], [], 'go', markersize=8, label='Start')
            self.end_marker, = self.ax.plot([], [], 'rx', markersize=8, label='End')
            self.current_marker, = self.ax.plot([], [], 'ro', markersize=6, label='Current')

            # Stats text box
            self.stats_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                         verticalalignment='top', bbox=dict(boxstyle='round',
                                         facecolor='wheat', alpha=0.8))

            self.ax.legend(loc='upper right')
            self.ax.set_xlim(-0.5, 0.5)
            self.ax.set_ylim(-0.5, 0.5)

            # Show plot window non-blocking
            plt.show(block=False)

            # Manual update loop instead of animation
            while self.running:
                self._manual_update()
                time.sleep(self.update_interval / 1000.0)  # Convert ms to seconds

        except Exception as e:
            print(f"Error in live plotter: {e}")
        finally:
            self.running = False

    def _manual_update(self):
        """Manual update method for the plot."""
        try:
            # Get current log count
            current_count = self.log_count_ref[0]

            if current_count <= 0:
                return

            # Extract position data
            x_data = self.logs[:current_count, 0]  # x coordinates
            y_data = self.logs[:current_count, 1]  # y coordinates

            # Filter out zero/invalid points
            valid_mask = ~((x_data == 0) & (y_data == 0))
            x_valid = x_data[valid_mask]
            y_valid = y_data[valid_mask]

            if len(x_valid) == 0:
                return

            # Update path line
            self.line.set_data(x_valid, y_valid)

            # Update markers
            if len(x_valid) >= 1:
                # Start marker
                self.start_marker.set_data([x_valid[0]], [y_valid[0]])

                # Current position marker
                self.current_marker.set_data([x_valid[-1]], [y_valid[-1]])

                if len(x_valid) > 1:
                    # End marker (only show if we have movement)
                    self.end_marker.set_data([x_valid[-1]], [y_valid[-1]])

            # Update plot limits with some padding
            if len(x_valid) > 0:
                x_margin = max(0.1, (np.max(x_valid) - np.min(x_valid)) * 0.1)
                y_margin = max(0.1, (np.max(y_valid) - np.min(y_valid)) * 0.1)

                self.ax.set_xlim(np.min(x_valid) - x_margin, np.max(x_valid) + x_margin)
                self.ax.set_ylim(np.min(y_valid) - y_margin, np.max(y_valid) + y_margin)

            # Calculate and update statistics
            stats = self._calculate_stats(x_valid, y_valid, current_count)
            self.stats_text.set_text(stats)

            # Redraw the plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            self.last_update_count = current_count

        except Exception as e:
            print(f"Error updating plot: {e}")


    def _calculate_stats(self, x, y, sample_count):
        """Calculate real-time path statistics."""
        if len(x) < 2:
            return f"Samples: {sample_count}\nPosition: ({x[-1]:.3f}, {y[-1]:.3f})" if len(x) > 0 else "Waiting for data..."

        # Calculate path length
        dx = np.diff(x)
        dy = np.diff(y)
        step_lengths = np.hypot(dx, dy)
        total_length = float(np.sum(step_lengths))

        # Calculate net drift
        net_drift = float(np.hypot(x[-1] - x[0], y[-1] - y[0]))

        # Calculate max radial deviation from start
        max_radius = float(np.max(np.hypot(x - x[0], y - y[0])))

        # Current position
        curr_x, curr_y = x[-1], y[-1]

        stats = f"""Samples: {sample_count}
Current: ({curr_x:.3f}, {curr_y:.3f})
Path Length: {total_length:.3f} m
Net Drift: {net_drift:.3f} m
Max Radius: {max_radius:.3f} m"""

        return stats


def create_shared_log_count_ref(log_count_var):
    """
    Create a shared reference to log_count that can be updated by the main thread.
    This is a simple wrapper to allow the plotter to access the current count.
    """
    return [log_count_var]


# Example usage (to be integrated with the main flight script)
if __name__ == "__main__":
    # This is just for testing the plotter independently
    print("Live Path Plotter - Test Mode")
    print("This module is designed to be imported and used with the flight script.")

    # Simulate some test data
    test_logs = np.zeros((1000, 9))
    test_count_ref = [0]

    # Create some sample path data
    t = np.linspace(0, 4*np.pi, 100)
    x_path = 0.5 * np.cos(t)
    y_path = 0.5 * np.sin(t)

    plotter = LivePathPlotter(test_logs, test_count_ref, update_interval=100)

    try:
        plotter.start()

        # Simulate data being added
        for i in range(len(x_path)):
            test_logs[i, 0] = x_path[i]  # x
            test_logs[i, 1] = y_path[i]  # y
            test_logs[i, 2] = 0.5       # z
            test_count_ref[0] = i + 1
            time.sleep(0.1)

        time.sleep(5)  # Show final result for a bit

    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        plotter.stop()