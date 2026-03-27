#!/usr/bin/env python3
"""
calibrate_tags.py — Inter-tag transform calibration tool for convoy_traj.

Collects N samples of the rigid transform between tag36h11:0 and tag36h11:1
while both are simultaneously visible to the camera, averages them, and writes
the result to a tag_calibration.yaml file that the convoy_traj_node can load.

Usage
-----
    ros2 run convoy_traj calibrate_tags.py --output /path/to/tag_calibration.yaml
    ros2 run convoy_traj calibrate_tags.py --output /path/to/tag_calibration.yaml --samples 200
    ros2 run convoy_traj calibrate_tags.py --output /path/to/tag_calibration.yaml --timeout 30.0

The output YAML is formatted as a ROS 2 parameter file that can be passed directly
to the convoy_traj_node via its launch file.

After running, rebuild the workspace so the file is installed to the share directory:
    colcon build --packages-select convoy_traj && source install/setup.bash
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer, TransformListener

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is not installed. Install with: pip install pyyaml", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Quaternion helpers (avoids a numpy dependency)
# ---------------------------------------------------------------------------

def quat_mul(q1, q2):
    """Multiply two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_normalize(q):
    """Normalize a quaternion (x, y, z, w)."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_average(quats):
    """
    Average a list of unit quaternions by component-wise mean + renormalization.

    Valid when all samples are close together (rigid transform between two fixed
    tags on the same vehicle face). For larger spreads use the eigenvector method.
    """
    # Ensure all quaternions are in the same hemisphere as the first one
    # to avoid cancellation artefacts.
    ref = quats[0]
    aligned = []
    for q in quats:
        dot = ref[0] * q[0] + ref[1] * q[1] + ref[2] * q[2] + ref[3] * q[3]
        if dot < 0.0:
            aligned.append((-q[0], -q[1], -q[2], -q[3]))
        else:
            aligned.append(q)

    n = len(aligned)
    avg = (
        sum(q[0] for q in aligned) / n,
        sum(q[1] for q in aligned) / n,
        sum(q[2] for q in aligned) / n,
        sum(q[3] for q in aligned) / n,
    )
    return quat_normalize(avg)


# ---------------------------------------------------------------------------
# Calibration node
# ---------------------------------------------------------------------------

class CalibrationNode(Node):
    """Short-lived node that collects inter-tag transform samples."""

    def __init__(self, tag0: str, tag1: str):
        super().__init__('tag_calibration_node')
        self.tag0 = tag0
        self.tag1 = tag1
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def try_lookup(self):
        """
        Attempt a single lookupTransform(tag0, tag1).

        Returns a dict with keys tx, ty, tz, qx, qy, qz, qw on success,
        or None on failure.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.tag0,
                self.tag1,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tr = t.transform
            return {
                'tx': tr.translation.x,
                'ty': tr.translation.y,
                'tz': tr.translation.z,
                'qx': tr.rotation.x,
                'qy': tr.rotation.y,
                'qz': tr.rotation.z,
                'qw': tr.rotation.w,
            }
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description='Calibrate inter-tag transform and save to YAML.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--output', required=True,
        help='Path to write tag_calibration.yaml (required).',
    )
    parser.add_argument(
        '--samples', type=int, default=100,
        help='Number of valid samples to collect (default: 100).',
    )
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='Maximum wall-clock seconds to wait for samples (default: 60).',
    )
    parser.add_argument(
        '--tag0', default='tag36h11:2',
        help='TF frame name of the first tag (default: tag36h11:2).',
    )
    parser.add_argument(
        '--tag1', default='tag36h11:3',
        help='TF frame name of the second tag (default: tag36h11:3).',
    )
    # Strip ROS 2 remapping args before parsing (ros2 run injects them)
    argv = [a for a in sys.argv[1:] if not a.startswith('__')]
    return parser.parse_args(argv)


def main():
    args = parse_args()

    # Validate output directory exists
    out_dir = os.path.dirname(os.path.abspath(args.output))
    if not os.path.isdir(out_dir):
        print(f"ERROR: output directory does not exist: {out_dir}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = CalibrationNode(tag0=args.tag0, tag1=args.tag1)

    print(f"[calibrate_tags] Collecting {args.samples} samples of "
          f"'{args.tag0}' -> '{args.tag1}' transform.")
    print(f"[calibrate_tags] Timeout: {args.timeout:.1f}s. "
          "Make sure both tags are visible to the camera.")

    samples = []
    deadline = time.monotonic() + args.timeout
    spin_rate_s = 0.02  # ~50 Hz

    while len(samples) < args.samples:
        if time.monotonic() > deadline:
            print(f"\nERROR: Timeout reached after {args.timeout:.1f}s. "
                  f"Collected {len(samples)}/{args.samples} samples.",
                  file=sys.stderr)
            rclpy.shutdown()
            sys.exit(1)

        rclpy.spin_once(node, timeout_sec=spin_rate_s)
        sample = node.try_lookup()

        if sample is not None:
            samples.append(sample)
            # Progress bar
            done = len(samples)
            bar_len = 40
            filled = int(bar_len * done / args.samples)
            bar = '#' * filled + '-' * (bar_len - filled)
            print(f"\r[{bar}] {done}/{args.samples}", end='', flush=True)
        else:
            remaining = deadline - time.monotonic()
            print(f"\r[calibrate_tags] Waiting for both tags... "
                  f"({remaining:.1f}s remaining)", end='', flush=True)

    print()  # newline after progress bar

    # ------------------------------------------------------------------
    # Average samples
    # ------------------------------------------------------------------
    n = len(samples)
    avg_tx = sum(s['tx'] for s in samples) / n
    avg_ty = sum(s['ty'] for s in samples) / n
    avg_tz = sum(s['tz'] for s in samples) / n

    quats = [(s['qx'], s['qy'], s['qz'], s['qw']) for s in samples]
    avg_q = quat_average(quats)

    print(f"[calibrate_tags] Averaged transform ({n} samples):")
    print(f"  translation : x={avg_tx:.6f}  y={avg_ty:.6f}  z={avg_tz:.6f}")
    print(f"  rotation    : qx={avg_q[0]:.6f}  qy={avg_q[1]:.6f}  "
          f"qz={avg_q[2]:.6f}  qw={avg_q[3]:.6f}")

    # ------------------------------------------------------------------
    # Write YAML (ROS 2 parameter file format)
    # ------------------------------------------------------------------
    calib_data = {
        'convoy_traj_node': {
            'ros__parameters': {
                'tag_calib_enabled': True,
                'tag_calib_tx': float(avg_tx),
                'tag_calib_ty': float(avg_ty),
                'tag_calib_tz': float(avg_tz),
                'tag_calib_qx': float(avg_q[0]),
                'tag_calib_qy': float(avg_q[1]),
                'tag_calib_qz': float(avg_q[2]),
                'tag_calib_qw': float(avg_q[3]),
                'tag_calib_samples': n,
            }
        }
    }

    out_path = os.path.abspath(args.output)
    with open(out_path, 'w') as f:
        f.write(
            f"# tag_calibration.yaml — generated by calibrate_tags.py\n"
            f"# Transform: '{args.tag0}' -> '{args.tag1}'\n"
            f"# Samples averaged: {n}\n"
            f"# Set tag_calib_enabled: false to disable single-tag fallback.\n\n"
        )
        yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)

    print(f"[calibrate_tags] Saved calibration to: {out_path}")
    print("[calibrate_tags] Rebuild the workspace to install the file:")
    print("  colcon build --packages-select convoy_traj && source install/setup.bash")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
