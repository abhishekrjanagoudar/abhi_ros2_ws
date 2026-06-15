#!/usr/bin/env python3
"""
data_recorder.py
----------------
Phase 1 of the imitation-learning pipeline.

Records synchronised LiDAR + velocity pairs while the operator drives the
robot manually with teleop.  Each row in driving_data.csv contains:

    lidar_0 … lidar_359  — 360 normalised LiDAR readings  (float, 0–1)
    linear_velocity       — linear.x from /diff_cont/cmd_vel  (m/s)
    angular_velocity      — angular.z from /diff_cont/cmd_vel  (rad/s)

Usage
-----
Terminal 1:  ros2 launch mobile_description robot.launch.py gz:=true
Terminal 2:  ros2 run mobile_description data_recorder.py
Terminal 3:  ros2 run mobile_description teleop_controller.py
Drive the robot, collect 3 000–10 000 samples, then Ctrl+C.
"""

import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------
TARGET_SIZE   = 360          # number of LiDAR points in every feature vector
FLUSH_EVERY   = 50           # flush CSV to disk every N samples
CSV_FILENAME  = "driving_data.csv"

# Column names written once as the CSV header
HEADER = (
    [f"lidar_{i}" for i in range(TARGET_SIZE)]
    + ["linear_velocity", "angular_velocity"]
)


# ===========================================================================
# Preprocessing
# ===========================================================================

def preprocess_scan(msg: LaserScan) -> np.ndarray:
    """Convert a raw LaserScan into a fixed-size, normalised numpy array.

    Steps
    -----
    1. Extract raw float32 range array from the message.
    2. Replace NaN / Inf / values outside [range_min, range_max] with
       max_range so invalid readings become "far away" rather than being
       discarded.  This keeps the array length stable.
    3. Clip to [0, max_range] — guards against slight sensor overshoots.
    4. If the scan does not contain exactly TARGET_SIZE (360) readings,
       interpolate linearly to TARGET_SIZE points so the neural network
       always sees a fixed-length input regardless of sensor configuration.
    5. Normalise to [0, 1] by dividing by max_range.

    Parameters
    ----------
    msg : sensor_msgs/LaserScan

    Returns
    -------
    np.ndarray, shape (TARGET_SIZE,), dtype float32, values in [0, 1]
    """
    max_r = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0 else 10.0

    # ── Step 1: raw array ─────────────────────────────────────────────────
    raw = np.array(msg.ranges, dtype=np.float32)

    # ── Step 2: sanitise invalid readings ────────────────────────────────
    # NaN, Inf, or anything outside [range_min, range_max] → max_r
    invalid = (
        ~np.isfinite(raw)
        | (raw < msg.range_min)
        | (raw > max_r)
    )
    raw[invalid] = max_r

    # ── Step 3: hard clip ─────────────────────────────────────────────────
    raw = np.clip(raw, 0.0, max_r)

    # ── Step 4: resize to TARGET_SIZE via linear interpolation ────────────
    # numpy.interp maps the old index domain onto the new one smoothly.
    if len(raw) != TARGET_SIZE:
        old_x = np.linspace(0.0, 1.0, len(raw))
        new_x = np.linspace(0.0, 1.0, TARGET_SIZE)
        raw   = np.interp(new_x, old_x, raw).astype(np.float32)

    # ── Step 5: normalise to [0, 1] ───────────────────────────────────────
    normalised = raw / max_r
    return normalised


# ===========================================================================
# CSV helper
# ===========================================================================

def save_sample(
    writer:   csv.writer,
    lidar:    np.ndarray,
    lin_vel:  float,
    ang_vel:  float,
) -> None:
    """Write one (lidar, velocity) pair as a row in the open CSV file.

    Parameters
    ----------
    writer  : csv.writer bound to the output file
    lidar   : float32 array of shape (TARGET_SIZE,), values in [0, 1]
    lin_vel : linear.x  in m/s
    ang_vel : angular.z in rad/s
    """
    row = lidar.tolist() + [float(lin_vel), float(ang_vel)]
    writer.writerow(row)


# ===========================================================================
# ROS 2 Node
# ===========================================================================

class DataRecorderNode(Node):
    """Subscribes to /scan and /diff_cont/cmd_vel, writes paired rows to CSV."""

    def __init__(self, csv_path: str):
        super().__init__("data_recorder")

        # ── Internal state ────────────────────────────────────────────────
        # Latest velocity command — updated by cmd_vel callback.
        # Synchronisation strategy: "last-known" pairing.
        # On every new scan we capture the most recent cmd_vel.
        # This is appropriate because teleop publishes at ~20 Hz and
        # LiDAR at 10 Hz; the velocity signal changes slowly relative
        # to the pairing window.
        self._latest_lin: float = 0.0
        self._latest_ang: float = 0.0
        self._sample_count: int = 0
        self._last_min_dist: float = 0.0

        # ── CSV setup ─────────────────────────────────────────────────────
        # Append mode: safe to stop and restart collection without losing data.
        # Header is written only when the file is being created for the first time.
        file_exists = os.path.isfile(csv_path)
        self._csv_file = open(csv_path, "a", newline="", buffering=1)
        self._writer   = csv.writer(self._csv_file)

        if not file_exists:
            self._writer.writerow(HEADER)
            self._csv_file.flush()
            self.get_logger().info(f"Created new CSV: {csv_path}")
        else:
            self.get_logger().info(f"Appending to existing CSV: {csv_path}")

        self._csv_path = csv_path

        # ── QoS profile ───────────────────────────────────────────────────
        # Best-effort matches Gazebo's sensor bridge; reliable matches teleop.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscription: /scan ───────────────────────────────────────────
        # Fires at the LiDAR update rate (10 Hz in simulation).
        # Each callback preprocesses one scan and saves a sample.
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            sensor_qos,
        )

        # ── Subscription: /diff_cont/cmd_vel ─────────────────────────────
        # Fires whenever teleop_controller.py publishes a velocity command.
        # We only cache the latest values; no complex time-sync needed here.
        self.create_subscription(
            TwistStamped,
            "/diff_cont/cmd_vel",
            self._cmd_vel_callback,
            sensor_qos,
        )

        self.get_logger().info(
            "DataRecorder ready. Drive the robot with teleop to collect data."
        )

    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: TwistStamped) -> None:
        """Cache the latest velocity command from teleop."""
        self._latest_lin = msg.twist.linear.x
        self._latest_ang = msg.twist.angular.z

    # ------------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan) -> None:
        """Process one LiDAR scan and save a training sample.

        Called at 10 Hz.  Pairs the scan with the last known velocity command.
        """
        # ── Preprocess scan ───────────────────────────────────────────────
        try:
            lidar_features = preprocess_scan(msg)
        except Exception as exc:
            self.get_logger().warn(f"preprocess_scan failed: {exc}")
            return

        # ── Track minimum distance for live display ────────────────────────
        # Use the raw ranges from the message so we show real metres, not normalised.
        raw_valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        self._last_min_dist = min(raw_valid) if raw_valid else 0.0

        # ── Save sample ───────────────────────────────────────────────────
        try:
            save_sample(
                self._writer,
                lidar_features,
                self._latest_lin,
                self._latest_ang,
            )
        except Exception as exc:
            self.get_logger().error(f"save_sample failed: {exc}")
            return

        self._sample_count += 1

        # ── Periodic flush ────────────────────────────────────────────────
        # Flush every FLUSH_EVERY samples to reduce I/O overhead while still
        # persisting data regularly in case of crash.
        if self._sample_count % FLUSH_EVERY == 0:
            self._csv_file.flush()

        # ── Live statistics ───────────────────────────────────────────────
        self._print_stats()

    # ------------------------------------------------------------------
    def _print_stats(self) -> None:
        """Overwrite the current terminal line with live recording statistics."""
        line = (
            f"\rSamples: {self._sample_count:>6}  |  "
            f"Linear: {self._latest_lin:+.2f} m/s  |  "
            f"Angular: {self._latest_ang:+.2f} rad/s  |  "
            f"Min dist: {self._last_min_dist:.2f} m   "
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    # ------------------------------------------------------------------
    def close(self) -> None:
        """Flush and close the CSV file on shutdown."""
        self._csv_file.flush()
        self._csv_file.close()
        print(f"\n[INFO] Saved {self._sample_count} samples → {self._csv_path}")


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    csv_path = os.path.join(os.getcwd(), CSV_FILENAME)
    node = DataRecorderNode(csv_path)

    print("=" * 60)
    print("  Imitation Learning — Phase 1: Data Recorder")
    print(f"  Output: {csv_path}")
    print("  Drive with teleop.  Ctrl+C to stop.")
    print("=" * 60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — shutting down.")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
