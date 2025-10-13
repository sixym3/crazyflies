#!/usr/bin/env python3
import csv
import math
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range

# NOTE: don't force a backend; let matplotlib pick a working interactive one.
import matplotlib.pyplot as plt


class RangeMonitorNode(Node):
    def __init__(self):
        super().__init__('range_monitor_node')

        # -------------------- Parameters --------------------
        self.declare_parameter('topic', '/crazyflie/range/down')
        self.declare_parameter('time_window_sec', 5.0)     # plot window
        self.declare_parameter('z_window_sec', 1.0)        # z-score baseline window (for derivative)
        self.declare_parameter('z_threshold', 3.0)         # edge detection threshold
        self.declare_parameter('expected_height_m', 0.30)  # for info/logging only
        self.declare_parameter('plot_update_hz', 20.0)     # how often to redraw plot
        self.declare_parameter('log_dir', '.')             # directory to drop CSV log

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.time_window = float(self.get_parameter('time_window_sec').value)
        self.z_window = float(self.get_parameter('z_window_sec').value)
        self.z_thresh = float(self.get_parameter('z_threshold').value)
        self.expected_height = float(self.get_parameter('expected_height_m').value)
        self.plot_update_hz = float(self.get_parameter('plot_update_hz').value)
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        # -------------------- QoS & Subscriber --------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.sub = self.create_subscription(Range, self.topic, self.range_cb, sensor_qos)

        # -------------------- Data Buffers --------------------
        # Absolute time (sec, from ROS clock) + values (m)
        self.times = deque()
        self.values = deque()
        # For derivative-based z-score
        self.delta_times = deque()   # times for deltas
        self.deltas = deque()        # dr/dt samples

        # -------------------- Plot Setup --------------------
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        (self.line,) = self.ax.plot([], [], linewidth=2)
        self.ax.set_xlabel('Time (s ago)')
        self.ax.set_ylabel('Range (m)')
        self.ax.set_title('Crazyflie Down Range — Last 5 s')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(-self.time_window, 0)
        self.ax.set_ylim(0.0, 2.0)

        # Show threshold/expected height as reference
        self.exp_height_line = self.ax.axhline(self.expected_height, linestyle='--', linewidth=1)

        # Text readouts
        self.text_info = self.ax.text(
            0.02, 0.95, '', transform=self.ax.transAxes, va='top', ha='left'
        )

        # -------------------- CSV Logging --------------------
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = f'{self.log_dir}/range_down_log_{ts}.csv'
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_ns',
            'time_iso',
            'range_m',
            'height_err_m',      # range - expected_height
            'delta_m_per_s',     # derivative estimate
            'zscore_delta',
            'edge_flag'          # 1 if |z| >= threshold else 0
        ])
        self.get_logger().info(f'Logging to {self.csv_path}')

        # -------------------- Timers --------------------
        # Plot update timer
        self.plot_timer = self.create_timer(1.0 / self.plot_update_hz, self.update_plot)

        self.get_logger().info(f'Range Monitor Node started. Listening on {self.topic}')

    # -------------------- Helpers --------------------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def prune_deque(self, dq, current_time, window):
        cutoff = current_time - window
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    # -------------------- Callback --------------------
    def range_cb(self, msg: Range):
        t = self.now_sec()

        # Sanitize the measurement (ignore inf/NaN, clip to msg.min/max if provided)
        r = float(msg.range)
        if math.isinf(r) or math.isnan(r):
            return
        # Optional clipping to declared sensor bounds
        if msg.min_range < msg.max_range:
            r = max(msg.min_range, min(msg.max_range, r))

        # Append raw measurement
        self.times.append((t,))   # store as tuple for uniformity with prune helper
        self.values.append((t, r))

        # Prune main window
        self.prune_old_data()

        # Compute derivative (simple backward diff)
        if len(self.values) >= 2:
            (t1, r1) = self.values[-1]
            (t0, r0) = self.values[-2]
            dt = max(1e-6, t1 - t0)
            drdt = (r1 - r0) / dt
            self.delta_times.append((t1,))
            self.deltas.append((t1, drdt))

            # Prune z-score window for derivative
            self.prune_deque(self.delta_times, t1, self.z_window)
            self.prune_deque(self.deltas, t1, self.z_window)

            # Compute z-score of current derivative against the last z_window seconds
            mean_d, std_d = self.mean_std_from(self.deltas)
            z = (drdt - mean_d) / std_d if std_d > 1e-9 else 0.0
            edge_flag = int(abs(z) >= self.z_thresh)
        else:
            drdt = 0.0
            z = 0.0
            edge_flag = 0

        # Log CSV
        time_ns = self.get_clock().now().nanoseconds
        time_iso = datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'
        height_err = r - self.expected_height
        self.csv_writer.writerow([time_ns, time_iso, f'{r:.6f}', f'{height_err:.6f}', f'{drdt:.6f}', f'{z:.6f}', edge_flag])

        # Optional console log when we think we hit an edge
        if edge_flag:
            self.get_logger().warn(
                f'EDGE candidate: Δrange={drdt:.3f} m/s, z={z:.2f}, range={r:.3f} m'
            )

    def prune_old_data(self):
        # keep only last time_window seconds in (times, values)
        if not self.times:
            return
        current_time = self.times[-1][0]
        cutoff = current_time - self.time_window
        while self.times and self.times[0][0] < cutoff:
            self.times.popleft()
        while self.values and self.values[0][0] < cutoff:
            self.values.popleft()

    @staticmethod
    def mean_std_from(dq_of_pairs):
        # dq_of_pairs stores (t, value)
        if not dq_of_pairs:
            return 0.0, 1.0
        vals = [v for (_, v) in dq_of_pairs]
        n = len(vals)
        mean_v = sum(vals) / n
        var = sum((v - mean_v) ** 2 for v in vals) / max(1, n - 1)
        std_v = math.sqrt(var)
        return mean_v, std_v

    # -------------------- Plot Update --------------------
    def update_plot(self):
        # nothing to draw yet
        if not self.values:
            plt.pause(0.001)
            return

        current_time = self.now_sec()
        # x-axis: seconds ago; y-axis: range
        xs = [t - current_time for (t, _) in self.values]
        ys = [r for (_, r) in self.values]

        self.line.set_data(xs, ys)
        self.ax.set_xlim(-self.time_window, 0)

        # auto y-limits with a little margin
        ymin = min(ys)
        ymax = max(ys)
        margin = max(0.05, 0.1 * (ymax - ymin))  # at least 5 cm or 10%
        lo = max(0.0, ymin - margin)
        hi = ymax + margin
        if hi <= lo:
            hi = lo + 0.1
        self.ax.set_ylim(lo, hi)

        # annotate status
        last_range = ys[-1]
        _, last_drdt = (self.deltas[-1] if self.deltas else (current_time, 0.0))
        mean_d, std_d = self.mean_std_from(self.deltas)
        z = (last_drdt - mean_d) / std_d if std_d > 1e-9 else 0.0
        self.text_info.set_text(
            f'range: {last_range:.3f} m   '
            f'Δrange: {last_drdt:.3f} m/s   '
            f'z(Δ): {z:.2f}   '
            f'edges @ |z| ≥ {self.z_thresh:g}'
        )

        self.fig.canvas.draw()
        # process GUI events (non-blocking)
        plt.pause(0.001)

    # -------------------- Shutdown --------------------
    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RangeMonitorNode()
    try:
        # show window without blocking; ROS timer handles redraws
        plt.ion()
        plt.show()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            plt.close('all')
        except Exception:
            pass


if __name__ == '__main__':
    main()
