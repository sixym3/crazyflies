#!/usr/bin/env python3
import logging
import time
import datetime as dt
import numpy as np
import os
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# Import live plotter
from live_path_plotter import LivePathPlotter

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# =========================
# Logging buffer (NumPy)
# =========================
LOG_COLUMNS = [
    "x", "y", "z", "yaw",
    "front", "back", "left", "right", "up"
]
INIT_ROWS = 100_000
logs = np.zeros((INIT_ROWS, len(LOG_COLUMNS)), dtype=float)
log_count = 0

# Shared reference for live plotter to access log_count
log_count_ref = [0]

def _ensure_capacity(next_index: int):
    """Grow the numpy buffer if we're about to exceed capacity."""
    global logs
    if next_index >= logs.shape[0]:
        new_rows = int(logs.shape[0] * 1.5)
        grown = np.zeros((new_rows, logs.shape[1]), dtype=float)
        grown[:logs.shape[0], :] = logs
        logs = grown

# =========================
# Callbacks
# =========================
def log_data_callback(timestamp, data, logconf):
    """
    Store data from the two log configs into the same row.
    We increment the row only after a Multiranger packet so each row
    contains one (latest) Position + the matching Multiranger sample.
    """
    global log_count, logs, log_count_ref

    # (Optional) uncomment if you still want console prints:
    print(f'[{timestamp}][{logconf.name}]: {data}')

    # Make sure there's room for this sample
    _ensure_capacity(log_count)

    if logconf.name == 'Position':
        logs[log_count, 0] = data.get('stateEstimate.x', 0.0)
        logs[log_count, 1] = data.get('stateEstimate.y', 0.0)
        logs[log_count, 2] = data.get('stateEstimate.z', 0.0)
        logs[log_count, 3] = data.get('stateEstimate.yaw', 0.0)

    elif logconf.name == 'Multiranger':
        logs[log_count, 4] = float(data.get('range.front', 0))
        logs[log_count, 5] = float(data.get('range.back', 0))
        logs[log_count, 6] = float(data.get('range.left', 0))
        logs[log_count, 7] = float(data.get('range.right', 0))
        logs[log_count, 8] = float(data.get('range.up', 0))
        log_count += 1  # advance only after multiranger to "commit" the row
        log_count_ref[0] = log_count  # Update shared reference for live plotter

def start_logging(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_data_callback)
    logconf.start()

def stop_logging(logconf):
    try:
        logconf.stop()
    except Exception:
        # If already stopped or CF disconnected, ignore
        pass

def save_logs():
    global logs, log_count
    # Timestamped filename
    filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
    # Ensure logs directory
    os.makedirs('logs', exist_ok=True)
    filepath = os.path.join(os.getcwd(), 'logs', filename)

    # Trim unused rows and save with header
    used = logs[:log_count]
    header = ",".join(LOG_COLUMNS)
    np.savetxt(filepath, used, delimiter=',', header=header, comments='')
    print(f"Logs saved to {filepath} ({used.shape[0]} rows).")

def simple_flight(enable_live_plot=True):
    """
    Execute a test flight pattern for occupancy grid data collection.

    Flight pattern:
    1. Take off and rise to 0.6m
    2. Move forward 1.0m
    3. Continuous 360-degree turn (30°/sec for 12 seconds) to scan environment
    4. Return to start position
    5. Land

    This pattern generates comprehensive multiranger data for testing occupancy grid visualization.
    """
    # Initialize the low-level drivers (before connecting)
    cflib.crtp.init_drivers()

    # Initialize live plotter if enabled
    live_plotter = None
    if enable_live_plot:
        live_plotter = LivePathPlotter(logs, log_count_ref, update_interval=200)

    # Log configs (100 ms = 10 Hz; adjust if needed)
    lg_position = LogConfig(name='Position', period_in_ms=100)
    lg_position.add_variable('stateEstimate.x', 'float')
    lg_position.add_variable('stateEstimate.y', 'float')
    lg_position.add_variable('stateEstimate.z', 'float')
    lg_position.add_variable('stateEstimate.yaw', 'float')

    lg_multiranger = LogConfig(name='Multiranger', period_in_ms=100)
    lg_multiranger.add_variable('range.front', 'uint16_t')
    lg_multiranger.add_variable('range.back', 'uint16_t')
    lg_multiranger.add_variable('range.left', 'uint16_t')
    lg_multiranger.add_variable('range.right', 'uint16_t')
    lg_multiranger.add_variable('range.up', 'uint16_t')

    # Start live plotter before connecting to drone
    if live_plotter:
        live_plotter.start()

    try:
        # Connect and keep logging running throughout the flight
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("Connected to Crazyflie!")

            # Start async logging BEFORE motion, keep running through the whole flight
            start_logging(scf, lg_position)
            start_logging(scf, lg_multiranger)
            print("Logging started...")

            try:
                with MotionCommander(scf) as mc:
                    print("Taking off...")

                    print("Going up 0.6 m...")
                    mc.up(0.6)
                    time.sleep(2)

                    print("Moving forward 1.0 m...")
                    mc.forward(1.0)
                    time.sleep(3)  # Pause to collect data at new position

                    print("Turning around continuously to scan environment...")
                    # Continuous turn for smooth data collection
                    # Turn at ~30 degrees/second for 12 seconds = 360 degrees
                    mc.start_turn_right(30)  # Start continuous right turn at 30 deg/sec
                    time.sleep(12)  # Turn for 12 seconds (360 degrees total)
                    mc.stop()  # Stop the turn

                    print("Turn complete. Returning to start position...")
                    mc.back(1.0)  # Return to starting position
                    time.sleep(2)

                    print("Landing...")
                print("Motion complete.")
            finally:
                # Always stop logging and save, even if motion throws
                stop_logging(lg_position)
                stop_logging(lg_multiranger)
                save_logs()

    finally:
        # Always stop live plotter
        if live_plotter:
            live_plotter.stop()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Crazyflie autonomous flight with optional live plotting')
    parser.add_argument('--no-live-plot', action='store_true',
                       help='Disable live plotting (default: enabled)')
    args = parser.parse_args()

    enable_live_plot = not args.no_live_plot
    if enable_live_plot:
        print("Starting test flight with live plotting enabled...")
        print("Flight pattern: Forward 1m → Continuous 360° turn → Return → Land")
    else:
        print("Starting test flight with live plotting disabled...")
        print("Flight pattern: Forward 1m → Continuous 360° turn → Return → Land")

    simple_flight(enable_live_plot=enable_live_plot)
    print("Test flight completed! Check logs directory for data to use with plot_path_2d.py")
