# convoy_traj

ROS 2 (Humble) C++17 package for tracking a front vehicle using two AprilTags
(`tag36h11` family). It averages the two tag poses via translation mean + quaternion
SLERP, applies an exponential moving average (EMA) filter, and broadcasts the result
as TF frames and `PoseWithCovarianceStamped` topics.

---

## Features

- **Dual-tag averaging** — translation mean + quaternion SLERP of two side-by-side
  AprilTags for a more robust pose estimate
- **Single-tag fallback** — when only one tag is visible, the other's pose is
  reconstructed from a pre-calibrated inter-tag transform so output is never
  interrupted
- **EMA filter** — configurable smoothing factor `alpha` on both translation and
  rotation (SLERP-based)
- **Configurable TF freshness threshold** — rejects stale cached transforms so a
  disappeared tag is detected correctly regardless of TF buffer age
- **Composable node** — built as both a shared library component and a standalone
  executable from the same source file

---

## Repository layout

```
convoy_traj/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── convoy_traj_params.yaml   # node parameters (frames, alpha, offsets, …)
│   ├── tag_calibration.yaml      # inter-tag calibration (written by calibrate_tags.py)
│   └── ukf.yaml                  # placeholder (currently empty)
├── include/convoy_traj/          # public headers (currently empty)
├── launch/
│   ├── convoy_traj.launch.py             # standalone, inline args
│   ├── convoy_traj_yaml.launch.py        # standalone, loads YAML config files
│   ├── combined_convoy_system.launch.py  # full system (ZED + AprilTag + node)
│   └── combined_convoy_composable.launch.py
├── scripts/
│   ├── calibrate_tags.py         # inter-tag calibration tool
│   └── test_tag_fallback.py      # fake TF publisher for testing fallback scenarios
└── src/
    └── convoy_traj_node.cpp      # sole C++ source file
```

---

## Build

All commands run from the **workspace root** (`ros2_ws/`).

```bash
source /opt/ros/humble/setup.bash

# Build this package only
colcon build --packages-select convoy_traj

# Source the workspace after building
source install/setup.bash
```

---

## Run

```bash
# Run the node directly (uses in-code defaults)
ros2 run convoy_traj convoy_traj_node

# Launch loading YAML config files (recommended)
ros2 launch convoy_traj convoy_traj_yaml.launch.py

# Launch with inline parameter overrides
ros2 launch convoy_traj convoy_traj.launch.py alpha:=0.8 axes_offset_x:=1.5

# Override a parameter at runtime
ros2 param set /convoy_traj_node alpha 0.5
```

---

## Parameters

All parameters are declared with in-code defaults and can be overridden via YAML or
command-line argument. The primary YAML files are `config/convoy_traj_params.yaml`
and `config/tag_calibration.yaml`.

### Frames

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_frame_id` | `string` | `zed_left_camera_optical_frame` | Source (camera) TF frame |
| `tag0_frame_id` | `string` | `tag36h11:0` | TF frame of the first AprilTag |
| `tag1_frame_id` | `string` | `tag36h11:1` | TF frame of the second AprilTag |

### Filtering

| Parameter | Type | Default | Description |
|---|---|---|---|
| `alpha` | `double` | `0.1` | EMA smoothing factor (0–1). Higher = more responsive, less smooth. YAML default is `0.8`. |

### Front-axle offset

| Parameter | Type | Default | Description |
|---|---|---|---|
| `axes_offset_x` | `double` | `0.0` | Forward offset from `front_vehicle` to `front_axes` (m) |
| `axes_offset_y` | `double` | `0.0` | Lateral offset (m) |
| `axes_offset_z` | `double` | `0.0` | Vertical offset (m) |

### TF freshness

| Parameter | Type | Default | Description |
|---|---|---|---|
| `tf_max_age_ms` | `int` | `1000` | Max age (ms) of a TF transform before the tag is treated as not visible. Must exceed your AprilTag detector's pipeline latency. Increase if you see spurious "stale" warnings when both tags are visible. |

### Inter-tag calibration

Loaded automatically from `config/tag_calibration.yaml` (written by `calibrate_tags.py`).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `tag_calib_enabled` | `bool` | `false` | Enable single-tag fallback |
| `tag_calib_tx/ty/tz` | `double` | `0.0` | Translation of tag1 in tag0's frame (m) |
| `tag_calib_qx/qy/qz/qw` | `double` | `0/0/0/1` | Rotation of tag1's frame in tag0's frame |
| `tag_calib_samples` | `int` | `0` | Number of samples averaged during calibration (informational) |

---

## Topics and TF frames

| Name | Type | Direction | Description |
|---|---|---|---|
| `/tag_pose` | `PoseWithCovarianceStamped` | publish | Filtered pose of front vehicle in camera frame |
| `/front_axes_pose` | `PoseWithCovarianceStamped` | publish | Pose of front axle offset point in camera frame |
| `front_vehicle` | TF dynamic | broadcast | `camera_frame_id` → `front_vehicle` |
| `front_axes` | TF static | broadcast | `front_vehicle` → `front_axes` offset |

---

## Inter-tag calibration and single-tag fallback

The two AprilTags are mounted rigidly on the same face of the front vehicle. Their
relative pose (tag0 → tag1) is captured once by the calibration script and stored in
`config/tag_calibration.yaml`. The node uses it to reconstruct a missing tag's pose
whenever only one tag is visible.

### Fallback behaviour

| Tags visible | Behaviour |
|---|---|
| Both | Average as normal |
| Only tag0 | Reconstruct tag1: `T_cam_tag1 = T_cam_tag0 ∘ T_tag0_tag1_calib` |
| Only tag1 | Reconstruct tag0: `T_cam_tag0 = T_cam_tag1 ∘ T_tag0_tag1_calib⁻¹` |
| Neither | `RCLCPP_WARN` + early return (no output published) |

A `RCLCPP_WARN` is emitted every cycle when a tag is reconstructed so it is visible
in logs. When `tag_calib_enabled: false` and only one tag is visible, the node also
warns and skips publishing.

### Calibration workflow

```bash
# 1. Build and source at least once
colcon build --packages-select convoy_traj && source install/setup.bash

# 2. Start the camera and AprilTag detector so that both tag TF frames are live.
#    Both tags MUST be simultaneously visible.

# 3. Run the calibration script
ros2 run convoy_traj calibrate_tags.py \
    --output ~/ros2_ws/src/convoy_traj/config/tag_calibration.yaml \
    --samples 100

# 4. Rebuild to install the updated YAML to the share directory
colcon build --packages-select convoy_traj && source install/setup.bash

# 5. Launch — tag_calib_enabled: true is picked up automatically
ros2 launch convoy_traj convoy_traj_yaml.launch.py
```

Optional calibration flags:

| Flag | Default | Description |
|---|---|---|
| `--samples N` | `100` | Frames to average |
| `--timeout S` | `60` | Max seconds to wait for both tags |
| `--tag0 FRAME` | `tag36h11:0` | Override tag0 frame name |
| `--tag1 FRAME` | `tag36h11:1` | Override tag1 frame name |

> **Note:** `config/tag_calibration.yaml` is overwritten by `colcon build` if you
> edit it only in the install directory. Always save calibration results to the
> source tree.

---

## Testing without a camera

`scripts/test_tag_fallback.py` publishes fake TF transforms so you can verify
fallback behaviour without a real camera or AprilTag detector:

```bash
# Publish both tags (normal operation)
ros2 run convoy_traj test_tag_fallback.py --scenario both

# Simulate tag1 being occluded
ros2 run convoy_traj test_tag_fallback.py --scenario tag0-only

# Simulate tag0 being occluded
ros2 run convoy_traj test_tag_fallback.py --scenario tag1-only

# Simulate both tags gone
ros2 run convoy_traj test_tag_fallback.py --scenario none

# Cycle through scenarios automatically
ros2 run convoy_traj test_tag_fallback.py --scenario cycle
```

---
## Troubleshooting

### "Lookup would require extrapolation into the future"
The node was using `get_clock()->now()` for `lookupTransform`, which fails because
AprilTag TF data is stamped at image-capture time (slightly in the past). This is
fixed — the node now uses `tf2::TimePoint()` (latest cached) and rejects stale data
via `tf_max_age_ms` instead.

### "tag transform is stale" warnings when tags ARE visible
Your AprilTag detector's pipeline latency exceeds `tf_max_age_ms`. Increase it:
```bash
ros2 param set /convoy_traj_node tf_max_age_ms 3000
# or set tf_max_age_ms: 3000 in convoy_traj_params.yaml
```
The stale-age warning message prints the exact age in seconds to help you choose a
value.

### Pose "stuck" after tag disappears
If the pose freezes instead of falling back, check that `tag_calib_enabled: true` in
`config/tag_calibration.yaml` (run the calibration workflow above if not).

### No output published at all
  design — enable calibration or ensure both tags are in view

---

## TODO

- [ ] Check for `tag_pose` correctness (aligned with front vehicle)
- [ ] Change frame `front_vehicle` to `front_tags`
- [ ] `front_vehicle` should be the front posterior axes (`tag_pose` should be this)
