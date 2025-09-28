"""
PyQt5 3D Visualization

3D pointcloud visualization using PyQt5 and VisPy for real-time display
of drone position, obstacles, and target position.
"""

import math
import sys
import numpy as np

try:
    from PyQt5 import QtCore, QtWidgets
    from vispy import scene
    from vispy.scene import visuals
    from vispy.scene.cameras import TurntableCamera
    DEPENDENCIES_AVAILABLE = True
except ImportError as e:
    print(f"PyQt5 3D visualization dependencies not available: {e}")
    DEPENDENCIES_AVAILABLE = False

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from config import *


if DEPENDENCIES_AVAILABLE:
    class PyQt5Canvas(scene.SceneCanvas):
        """3D visualization canvas using VisPy."""

        def __init__(self, keyupdateCB, autonomous_navigator):
            scene.SceneCanvas.__init__(self, keys=None)
            self.size = CANVAS_SIZE
            self.unfreeze()
            self.view = self.central_widget.add_view()
            self.view.bgcolor = '#ffffff'
            self.view.camera = TurntableCamera(
                fov=10.0, distance=CAMERA_DISTANCE, up='+z', center=(0.0, 0.0, 0.0))

            # Initialize data arrays
            self.last_pos = [0, 0, 0]
            self.pos_markers = visuals.Markers()
            self.meas_markers = visuals.Markers()
            self.target_marker = visuals.Markers()
            self.pos_data = np.array([0, 0, 0], ndmin=2)
            self.meas_data = np.array([0, 0, 0], ndmin=2)
            self.target_data = np.array([2, 0, 0], ndmin=2)
            self.lines = []
            self.autonomous_navigator = autonomous_navigator

            # Add visual elements to scene
            self.view.add(self.pos_markers)
            self.view.add(self.meas_markers)
            self.view.add(self.target_marker)
            for i in range(6):
                line = visuals.Line()
                self.lines.append(line)
                self.view.add(line)

            # Initialize target marker
            self.target_marker.set_data(self.target_data, face_color='green', size=10, edge_color='darkgreen')

            self.keyCB = keyupdateCB
            self.freeze()

            # Add coordinate axes
            scene.visuals.XYZAxis(parent=self.view.scene)

        def on_key_press(self, event):
            """Handle key press events."""
            if not event.native.isAutoRepeat():
                if event.native.key() == QtCore.Qt.Key_Left:
                    self.keyCB('y', 1)
                elif event.native.key() == QtCore.Qt.Key_Right:
                    self.keyCB('y', -1)
                elif event.native.key() == QtCore.Qt.Key_Up:
                    self.keyCB('x', 1)
                elif event.native.key() == QtCore.Qt.Key_Down:
                    self.keyCB('x', -1)
                elif event.native.key() == QtCore.Qt.Key_A:
                    self.keyCB('yaw', -70)
                elif event.native.key() == QtCore.Qt.Key_D:
                    self.keyCB('yaw', 70)
                elif event.native.key() == QtCore.Qt.Key_Z:
                    self.keyCB('yaw', -200)
                elif event.native.key() == QtCore.Qt.Key_X:
                    self.keyCB('yaw', 200)
                elif event.native.key() == QtCore.Qt.Key_W:
                    self.keyCB('height', 0.1)
                elif event.native.key() == QtCore.Qt.Key_S:
                    self.keyCB('height', -0.1)
                elif event.native.key() == QtCore.Qt.Key_M:
                    self.keyCB('autonomous_toggle', 1)

        def on_key_release(self, event):
            """Handle key release events."""
            if not event.native.isAutoRepeat():
                if event.native.key() in [QtCore.Qt.Key_Left, QtCore.Qt.Key_Right]:
                    self.keyCB('y', 0)
                elif event.native.key() in [QtCore.Qt.Key_Up, QtCore.Qt.Key_Down]:
                    self.keyCB('x', 0)
                elif event.native.key() in [QtCore.Qt.Key_A, QtCore.Qt.Key_D, QtCore.Qt.Key_Z, QtCore.Qt.Key_X]:
                    self.keyCB('yaw', 0)
                elif event.native.key() in [QtCore.Qt.Key_W, QtCore.Qt.Key_S]:
                    self.keyCB('height', 0)
                elif event.native.key() == QtCore.Qt.Key_M:
                    self.keyCB('autonomous_toggle', 0)

        def set_position(self, pos):
            """Update drone position."""
            self.last_pos = pos
            if PLOT_CRAZYFLIE:
                self.pos_data = np.append(self.pos_data, [pos], axis=0)
                self.pos_markers.set_data(self.pos_data, face_color='red', size=5)

            # Update target position based on mode
            if self.autonomous_navigator.target_world_position is None:
                # Manual mode: target follows drone position
                self.update_target_position(pos)
            else:
                # Autonomous mode: use fixed target
                self.update_target_position(pos)

        def update_target_position(self, drone_pos):
            """Update target marker using fixed target or relative position."""
            if self.autonomous_navigator.target_world_position is not None:
                target_x, target_y, target_z = self.autonomous_navigator.target_world_position
            else:
                # Default behavior: 2m forward from current drone position
                target_x = drone_pos[0] + 2.0
                target_y = drone_pos[1]
                target_z = drone_pos[2]

            self.target_data = np.array([[target_x, target_y, target_z]])
            self.target_marker.set_data(self.target_data, face_color='green', size=10, edge_color='darkgreen')

        def rot(self, roll, pitch, yaw, origin, point):
            """Rotate point around origin."""
            cosr = math.cos(math.radians(roll))
            cosp = math.cos(math.radians(pitch))
            cosy = math.cos(math.radians(yaw))

            sinr = math.sin(math.radians(roll))
            sinp = math.sin(math.radians(pitch))
            siny = math.sin(math.radians(yaw))

            roty = np.array([[cosy, -siny, 0],
                           [siny, cosy, 0],
                           [0, 0, 1]])

            rotp = np.array([[cosp, 0, sinp],
                           [0, 1, 0],
                           [-sinp, 0, cosp]])

            rotr = np.array([[1, 0, 0],
                           [0, cosr, -sinr],
                           [0, sinr, cosr]])

            rotFirst = np.dot(rotr, rotp)
            rot = np.array(np.dot(rotFirst, roty))
            tmp = np.subtract(point, origin)
            tmp2 = np.dot(rot, tmp)
            return np.add(tmp2, origin)

        def rotate_and_create_points(self, measurements):
            """Create 3D points from sensor measurements."""
            data = []
            o = self.last_pos
            roll = measurements['roll']
            pitch = -measurements['pitch']
            yaw = measurements['yaw']

            def add_point_if_near(point):
                # Calculate distance from drone to obstacle point
                dx = point[0] - o[0]
                dy = point[1] - o[1]
                distance = math.sqrt(dx * dx + dy * dy)

                # Exclude points greater than 2 meters
                if distance > 2.0:
                    return False

                rotated_point = self.rot(roll, pitch, yaw, o, point)
                data.append(rotated_point)
                return True

            # Process sensor measurements
            if measurements['up'] < SENSOR_THRESHOLD_MM:
                up = [o[0], o[1], o[2] + measurements['up'] / 1000.0]
                add_point_if_near(up)

            if measurements['down'] < SENSOR_THRESHOLD_MM and PLOT_SENSOR_DOWN:
                down = [o[0], o[1], o[2] - measurements['down'] / 1000.0]
                add_point_if_near(down)

            if measurements['left'] < SENSOR_THRESHOLD_MM:
                left = [o[0], o[1] + measurements['left'] / 1000.0, o[2]]
                add_point_if_near(left)

            if measurements['right'] < SENSOR_THRESHOLD_MM:
                right = [o[0], o[1] - measurements['right'] / 1000.0, o[2]]
                add_point_if_near(right)

            if measurements['front'] < SENSOR_THRESHOLD_MM:
                front = [o[0] + measurements['front'] / 1000.0, o[1], o[2]]
                add_point_if_near(front)

            if measurements['back'] < SENSOR_THRESHOLD_MM:
                back = [o[0] - measurements['back'] / 1000.0, o[1], o[2]]
                add_point_if_near(back)

            return data

        def set_measurement(self, measurements):
            """Update visualization with new sensor measurements."""
            data = self.rotate_and_create_points(measurements)
            o = self.last_pos

            # Update sensor lines
            for i in range(6):
                if i < len(data):
                    self.lines[i].set_data(np.array([o, data[i]]))
                else:
                    self.lines[i].set_data(np.array([o, o]))

            # Update measurement markers
            if len(data) > 0:
                self.meas_data = np.append(self.meas_data, data, axis=0)
            self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)


    class PyQt5Visualizer:
        """Main PyQt5 3D visualization window."""

        def __init__(self, drone_controller):
            """
            Initialize PyQt5 3D visualizer.

            Args:
                drone_controller: DroneController instance
            """
            self.drone_controller = drone_controller
            self.app = QtWidgets.QApplication(sys.argv)
            self.window = QtWidgets.QMainWindow()

            # Setup window
            self.window.resize(700, 500)
            self.window.setWindowTitle('Crazyflie 3D Visualization - Manual Mode (Press M to toggle autonomous)')

            # Create canvas
            self.canvas = PyQt5Canvas(self._handle_key_input, drone_controller.autonomous_navigator)
            self.canvas.create_native()
            self.canvas.native.setParent(self.window)
            self.window.setCentralWidget(self.canvas.native)

            # Add callbacks to drone controller
            self.drone_controller.add_position_callback(self.canvas.set_position)
            self.drone_controller.add_sensor_callback(self.canvas.set_measurement)
            self.drone_controller.add_connection_callback(self._connection_callback)

        def _handle_key_input(self, key, value):
            """Handle keyboard input."""
            if key == 'autonomous_toggle':
                # Toggle autonomous mode
                new_state = self.drone_controller.toggle_autonomous_mode()
                mode_text = "Autonomous Mode" if new_state else "Manual Mode"
                self.window.setWindowTitle(f'Crazyflie 3D Visualization - {mode_text} (Press M to toggle)')
            else:
                # Update manual control
                self.drone_controller.update_manual_control(key, value)

        def _connection_callback(self, connected, uri):
            """Handle connection status changes."""
            if connected:
                print(f"3D Visualizer: Connected to {uri}")
            else:
                print(f"3D Visualizer: Disconnected from {uri}")

        def show(self):
            """Show the visualization window."""
            self.window.show()

        def run(self):
            """Run the visualization (blocking call)."""
            self.window.show()
            return self.app.exec_()

        def close(self):
            """Close the visualization."""
            self.window.close()

else:
    class PyQt5Visualizer:
        """Dummy class when PyQt5 is not available."""

        def __init__(self, *args, **kwargs):
            raise ImportError("PyQt5 visualization not available - missing dependencies")

        def show(self):
            pass

        def run(self):
            return 0

        def close(self):
            pass