#!/usr/bin/env python3
"""
data_collector.py
-----------------
ROS 2 Python node — collects labelled LiDAR scan data for obstacle
classification training (Step 1 of the ML pipeline).

Convention (ROS standard):
    index 0   = East  (0°)
    index 90  = North (90°)
    index 180 = West  (180°)
    index 270 = South (270°)
    Direction: counter-clockwise

Features (11 total):
    min_range   — closest reading across all 360 rays
    mean_range  — mean distance across valid rays
    std_range   — standard deviation of valid rays
    east_mean   — mean of indices   0–44   (East sector)
    ne_mean     — mean of indices  45–89   (North-East sector)
    north_mean  — mean of indices  90–134  (North sector)
    nw_mean     — mean of indices 135–179  (North-West sector)
    west_mean   — mean of indices 180–224  (West sector)
    sw_mean     — mean of indices 225–269  (South-West sector)
    south_mean  — mean of indices 270–314  (South sector)
    se_mean     — mean of indices 315–359  (South-East sector)

Labels:
    0 = open space
    1 = corridor
    2 = near wall
    3 = corner
    4 = obstacle cluster

Usage:
    ros2 run mobile_description data_collector.py

Output:
    scan_data.csv  (created fresh in the current working directory;
                    any existing file is deleted on startup)
"""

import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CSV_FILENAME = "scan_data.csv"

CSV_HEADER = [
    "min_range", "mean_range", "std_range",
    "east_mean", "ne_mean", "north_mean", "nw_mean",
    "west_mean", "sw_mean", "south_mean", "se_mean",
    "label",
]

# Sector definitions: (name, start_index, end_index)
# Each sector covers exactly 45 indices (45°).
SECTORS = [
    ("east",  0,   44),
    ("ne",    45,  89),
    ("north", 90,  134),
    ("nw",    135, 179),
    ("west",  180, 224),
    ("sw",    225, 269),
    ("south", 270, 314),
    ("se",    315, 359),
]

VALID_LABELS = {"0", "1", "2", "3", "4"}
LABEL_NAMES  = {
    "0": "open space",
    "1": "corridor",
    "2": "near wall",
    "3": "corner",
    "4": "obstacle cluster",
}

MAX_RANGE = 10.0   # clip ceiling (metres)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _sector_mean(ranges: list, start: int, end: int) -> float:
    """Mean of ranges[start:end+1], excluding inf/nan. Returns 0.0 if all invalid."""
    vals = [r for r in ranges[start: end + 1] if math.isfinite(r)]
    return float(sum(vals) / len(vals)) if vals else 0.0


def _fmt(value: float) -> str:
    """Format a range value for display."""
    return f"{value:.2f}m"


def _print_compass(sectors: dict) -> None:
    """
    Print a fixed-width ASCII compass rose showing all 8 sector means.

        sectors  — dict keyed by sector name (lowercase), value = float metres
    """
    N  = _fmt(sectors["north"])
    NE = _fmt(sectors["ne"])
    NW = _fmt(sectors["nw"])
    E  = _fmt(sectors["east"])
    W  = _fmt(sectors["west"])
    SE = _fmt(sectors["se"])
    SW = _fmt(sectors["sw"])
    S  = _fmt(sectors["south"])

    print("")
    print(f"           N: {N}")
    print(f"  NW: {NW:<8}   NE: {NE}")
    print(f"W: {W:<10}       E: {E}")
    print(f"  SW: {SW:<8}   SE: {SE}")
    print(f"           S: {S}")
    print("")


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class DataCollectorNode(Node):

    def __init__(self, csv_path: str):
        super().__init__("data_collector")

        self._csv_path = csv_path
        self._latest_features = None   # populated by scan callback

        # ── CSV setup ──────────────────────────────────────────────────────
        # Always start fresh — delete existing file so column headers match.
        if os.path.isfile(csv_path):
            os.remove(csv_path)
            self.get_logger().info(f"Deleted old CSV: {csv_path}")

        self._csv_file = open(csv_path, "w", newline="")
        self._writer   = csv.writer(self._csv_file)
        self._writer.writerow(CSV_HEADER)
        self._csv_file.flush()

        self.get_logger().info(f"CSV created → {csv_path}")

        # ── Subscriber ─────────────────────────────────────────────────────
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            10,
        )

        self.get_logger().info("Subscribed to /scan — waiting for scans…")

    # ------------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan) -> None:
        """Compute all 11 features from one LaserScan message."""

        ranges = list(msg.ranges)

        if len(ranges) < 360:
            self.get_logger().warn(
                f"Expected 360 ranges, got {len(ranges)} — skipping."
            )
            return

        # ── Global stats (all valid rays, clipped to MAX_RANGE) ───────────
        valid_all = [
            min(r, MAX_RANGE)
            for r in ranges
            if math.isfinite(r)
        ]

        if not valid_all:
            self.get_logger().warn("All ranges invalid — skipping.")
            return

        min_range  = min(valid_all)
        mean_range = sum(valid_all) / len(valid_all)
        variance   = sum((r - mean_range) ** 2 for r in valid_all) / len(valid_all)
        std_range  = math.sqrt(variance)

        # ── 8 sector means ────────────────────────────────────────────────
        sector_vals = {
            name: _sector_mean(ranges, start, end)
            for name, start, end in SECTORS
        }

        # Store as flat tuple matching CSV_HEADER order (minus label)
        self._latest_features = (
            min_range, mean_range, std_range,
            sector_vals["east"],
            sector_vals["ne"],
            sector_vals["north"],
            sector_vals["nw"],
            sector_vals["west"],
            sector_vals["sw"],
            sector_vals["south"],
            sector_vals["se"],
        )
        self._latest_sectors = sector_vals   # keep dict for compass display

    # ------------------------------------------------------------------
    def prompt_and_save(self) -> None:
        """Wait for a fresh scan, display features + compass, prompt label, save."""

        # Wait up to 5 s for a scan
        deadline_ns = self.get_clock().now().nanoseconds + 5_000_000_000

        while self._latest_features is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline_ns:
                print("\n[WARN] No scan received — is Gazebo running?")
                return

        # Snapshot (new scans may arrive during input())
        feats   = self._latest_features
        sectors = self._latest_sectors
        self._latest_features = None

        # ── Print scalar features ──────────────────────────────────────────
        print("\n─── Scan features ────────────────────────────────────────")
        print(f"  min_range  = {feats[0]:.3f} m")
        print(f"  mean_range = {feats[1]:.3f} m")
        print(f"  std_range  = {feats[2]:.3f} m")

        # ── ASCII compass ──────────────────────────────────────────────────
        _print_compass(sectors)

        # ── Label prompt ───────────────────────────────────────────────────
        print("  Labels:")
        for k, v in LABEL_NAMES.items():
            print(f"    {k} = {v}")

        label_str = input("\nEnter label [0-4] : ").strip()

        if label_str not in VALID_LABELS:
            print(f"[SKIP] Invalid label '{label_str}'. Row not saved.")
            return

        # ── Write to CSV ───────────────────────────────────────────────────
        self._writer.writerow([*feats, int(label_str)])
        self._csv_file.flush()

        print(
            f"[SAVED] label={label_str} ({LABEL_NAMES[label_str]})  →  {self._csv_path}"
        )

    # ------------------------------------------------------------------
    def close(self) -> None:
        """Flush and close the CSV file cleanly."""
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info("CSV file closed.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    csv_path = os.path.join(os.getcwd(), CSV_FILENAME)

    node = DataCollectorNode(csv_path)

    print("\n====================================================")
    print("  LiDAR Obstacle Data Collector  (11 features)")
    print("  Labels: 0=open  1=corridor  2=wall  3=corner  4=obstacle")
    print("  Press Ctrl+C to stop.")
    print("====================================================\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.prompt_and_save()

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — shutting down.")

    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
