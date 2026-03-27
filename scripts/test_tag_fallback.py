#!/usr/bin/env python3
"""
test_tag_fallback.py — Manual integration test for convoy_traj single-tag fallback.

Publishes fake TF transforms for tag36h11:0 and/or tag36h11:1 so you can verify
convoy_traj_node behaviour without a real camera.

Scenarios
---------
  both      Publish both tags every cycle (normal operation)
  tag0-only Publish only tag36h11:0 (tag1 must be reconstructed via calibration)
  tag1-only Publish only tag36h11:1 (tag0 must be reconstructed via calibration)
  none      Publish neither tag (node should stay silent)
  cycle     Rotate through: both → tag0-only → tag1-only → none (5 s each)

Usage
-----
  # Terminal 1 — start the node (calibration YAML must be installed)
  ros2 launch convoy_traj convoy_traj_yaml.launch.py

  # Terminal 2 — run this test script
  ros2 run convoy_traj test_tag_fallback.py --scenario both
  ros2 run convoy_traj test_tag_fallback.py --scenario tag0-only
  ros2 run convoy_traj test_tag_fallback.py --scenario cycle

  # Terminal 3 — observe output
  ros2 topic echo /tag_pose
  ros2 topic hz  /tag_pose
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ---------------------------------------------------------------------------
# Fake tag poses (in zed_left_camera_optical_frame)
# Edit these to match a realistic geometry for your setup.
# ---------------------------------------------------------------------------

# tag0: 1 m in front of the camera, 0.1 m to the left
TAG0_T = (1.0, 0.1, 0.0)
# Identity rotation — tag faces the camera straight on
TAG0_Q = (0.0, 0.0, 0.0, 1.0)

# tag1: 1 m in front, 0.1 m to the right (i.e. 0.2 m offset from tag0 in Y)
TAG1_T = (1.0, -0.1, 0.0)
TAG1_Q = (0.0, 0.0, 0.0, 1.0)

CAMERA_FRAME = "zed_left_camera_optical_frame"
TAG0_FRAME   = "tag36h11:0"
TAG1_FRAME   = "tag36h11:1"


def make_transform(parent: str, child: str,
                   t: tuple, q: tuple,
                   stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = t[0]
    msg.transform.translation.y = t[1]
    msg.transform.translation.z = t[2]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg


class FakeTagPublisher(Node):
    def __init__(self, scenario: str, period_s: float):
        super().__init__('fake_tag_publisher')
        self.scenario = scenario
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_wall_timer(
            period_s,
            self.publish_transforms,
        )
        self._cycle_start = time.monotonic()
        self._cycle_phases = ['both', 'tag0-only', 'tag1-only', 'none']
        self._phase_duration = 5.0  # seconds per phase in cycle mode
        self.get_logger().info(
            f"FakeTagPublisher started — scenario='{scenario}', "
            f"period={period_s:.3f}s"
        )

    def _current_scenario(self) -> str:
        if self.scenario != 'cycle':
            return self.scenario
        elapsed = time.monotonic() - self._cycle_start
        phase_idx = int(elapsed / self._phase_duration) % len(self._cycle_phases)
        phase = self._cycle_phases[phase_idx]
        remaining = self._phase_duration - (elapsed % self._phase_duration)
        self.get_logger().info(
            f"[cycle] phase='{phase}'  next phase in {remaining:.1f}s",
            throttle_duration_sec=1.0,
        )
        return phase

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()
        scenario = self._current_scenario()

        transforms = []
        if scenario in ('both', 'tag0-only'):
            transforms.append(
                make_transform(CAMERA_FRAME, TAG0_FRAME, TAG0_T, TAG0_Q, now)
            )
        if scenario in ('both', 'tag1-only'):
            transforms.append(
                make_transform(CAMERA_FRAME, TAG1_FRAME, TAG1_T, TAG1_Q, now)
            )

        if transforms:
            self.broadcaster.sendTransform(transforms)
        elif scenario == 'none':
            self.get_logger().warn(
                "Publishing NO tags — /tag_pose should be silent.",
                throttle_duration_sec=2.0,
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Publish fake AprilTag TF frames for convoy_traj testing.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--scenario',
        choices=['both', 'tag0-only', 'tag1-only', 'none', 'cycle'],
        default='cycle',
        help=(
            'Which tags to publish. '
            '"cycle" rotates through all scenarios every 5 s (default).'
        ),
    )
    parser.add_argument(
        '--rate', type=float, default=10.0,
        help='Publish rate in Hz (default: 10).',
    )
    # Strip ROS 2 remapping args injected by ros2 run
    argv = [a for a in sys.argv[1:] if not a.startswith('__')]
    return parser.parse_args(argv)


def main():
    args = parse_args()
    rclpy.init()
    node = FakeTagPublisher(scenario=args.scenario, period_s=1.0 / args.rate)

    print(f"\n[test_tag_fallback] Scenario: '{args.scenario}'")
    print("[test_tag_fallback] Expected behaviour:")
    if args.scenario == 'both':
        print("  /tag_pose published at ~10 Hz — midpoint of tag0 and tag1")
    elif args.scenario == 'tag0-only':
        print("  If calibrated:   /tag_pose published — tag1 reconstructed from tag0")
        print("  If uncalibrated: /tag_pose silent, RCLCPP_WARN in node log")
    elif args.scenario == 'tag1-only':
        print("  If calibrated:   /tag_pose published — tag0 reconstructed from tag1")
        print("  If uncalibrated: /tag_pose silent, RCLCPP_WARN in node log")
    elif args.scenario == 'none':
        print("  /tag_pose silent — no tags visible")
    elif args.scenario == 'cycle':
        print("  Rotates through all scenarios every 5 s")
        print("  Watch /tag_pose stop/start and RCLCPP_WARN appear/disappear")
    print()
    print("  Monitor: ros2 topic echo /tag_pose")
    print("  Monitor: ros2 topic hz   /tag_pose\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
