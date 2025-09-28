"""
Core Crazyflie drone controller

Handles basic drone communication, logging, and manual control commands.
Separated from visualization and autonomous navigation for modularity.
"""

import logging
import sys
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import sys
from pathlib import Path

# Add package root to path
package_root = Path(__file__).parent.parent
sys.path.insert(0, str(package_root))

from config import *


class DroneController:
    """Core drone communication and control functionality."""

    def __init__(self, uri=None, autonomous_navigator=None):
        """
        Initialize drone controller.

        Args:
            uri: Crazyflie URI (uses default if None)
            autonomous_navigator: Optional autonomous navigator instance
        """
        self.uri = uri or DEFAULT_URI
        self.autonomous_navigator = autonomous_navigator

        # Initialize Crazyflie
        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Control state
        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}
        self.is_connected = False
        self.last_position = [0.0, 0.0, 0.0]

        # Callbacks
        self.position_callbacks = []
        self.sensor_callbacks = []
        self.connection_callbacks = []

        # Setup callbacks
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)

    def connect(self):
        """Connect to the Crazyflie."""
        print(f"Connecting to Crazyflie at {self.uri}...")
        self.cf.open_link(self.uri)

    def disconnect(self):
        """Disconnect from the Crazyflie."""
        if self.cf is not None:
            self.cf.close_link()

    def add_position_callback(self, callback):
        """Add callback for position updates."""
        self.position_callbacks.append(callback)

    def add_sensor_callback(self, callback):
        """Add callback for sensor updates."""
        self.sensor_callbacks.append(callback)

    def add_connection_callback(self, callback):
        """Add callback for connection events."""
        self.connection_callbacks.append(callback)

    def update_manual_control(self, axis, value):
        """
        Update manual control input.

        Args:
            axis: 'x', 'y', 'yaw', or 'height'
            value: Control value
        """
        if axis in self.hover:
            if axis != 'height':
                self.hover[axis] = value * SPEED_FACTOR
            else:
                self.hover[axis] += value

    def send_control_command(self):
        """Send control command to drone (manual + autonomous)."""
        if not self.is_connected:
            return

        # Check if there's manual input
        has_manual_input = (self.hover['x'] != 0.0 or
                           self.hover['y'] != 0.0 or
                           self.hover['yaw'] != 0.0)

        # Use manual input if present, otherwise autonomous
        if has_manual_input:
            x, y, yaw = self.hover['x'], self.hover['y'], self.hover['yaw']
        else:
            # Get autonomous commands if available
            if self.autonomous_navigator:
                auto_cmd = self.autonomous_navigator.get_navigation_command()
                x, y, yaw = auto_cmd['x'], auto_cmd['y'], auto_cmd['yaw']
            else:
                x, y, yaw = 0.0, 0.0, 0.0

        # Send hover command
        self.cf.commander.send_hover_setpoint(x, y, yaw, self.hover['height'])

    def toggle_autonomous_mode(self):
        """Toggle autonomous navigation mode."""
        if self.autonomous_navigator:
            new_state = self.autonomous_navigator.set_enabled(
                not self.autonomous_navigator.enabled)
            return new_state
        return False

    def _connected(self, uri):
        """Called when Crazyflie connects."""
        print(f'Connected to {uri}')
        self.is_connected = True

        # Notify callbacks
        for callback in self.connection_callbacks:
            callback(True, uri)

        # Setup logging
        self._setup_position_logging()
        self._setup_sensor_logging()

    def _disconnected(self, uri):
        """Called when Crazyflie disconnects."""
        print(f'Disconnected from {uri}')
        self.is_connected = False

        # Notify callbacks
        for callback in self.connection_callbacks:
            callback(False, uri)

    def _setup_position_logging(self):
        """Setup position logging."""
        lpos = LogConfig(name='Position', period_in_ms=LOG_PERIOD)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self._position_callback)
            lpos.start()
        except KeyError as e:
            print(f'Could not start position logging: {str(e)} not found in TOC')
        except AttributeError:
            print('Could not add position log config, bad configuration.')

    def _setup_sensor_logging(self):
        """Setup sensor logging."""
        lmeas = LogConfig(name='Measurements', period_in_ms=LOG_PERIOD)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')

        try:
            self.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self._sensor_callback)
            lmeas.start()
        except KeyError as e:
            print(f'Could not start sensor logging: {str(e)} not found in TOC')
        except AttributeError:
            print('Could not add sensor log config, bad configuration.')

    def _position_callback(self, timestamp, data, logconf):
        """Handle position data."""
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.last_position = position

        # Update autonomous navigator
        if self.autonomous_navigator:
            self.autonomous_navigator.update_position(position)

        # Notify callbacks
        for callback in self.position_callbacks:
            callback(position)

    def _sensor_callback(self, timestamp, data, logconf):
        """Handle sensor data."""
        measurements = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right']
        }

        # Update autonomous navigator
        if self.autonomous_navigator:
            self.autonomous_navigator.update_sensors(measurements, self.last_position)

        # Notify callbacks
        for callback in self.sensor_callbacks:
            callback(measurements)

    def get_status(self):
        """Get current drone status."""
        return {
            'connected': self.is_connected,
            'position': self.last_position.copy(),
            'hover_commands': self.hover.copy(),
            'autonomous_enabled': (self.autonomous_navigator.enabled
                                 if self.autonomous_navigator else False)
        }