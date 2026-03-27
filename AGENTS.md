# AGENTS.md — convoy_traj

ROS 2 C++17 package that tracks a front vehicle using two AprilTags (tag36h11:0 and
tag36h11:1), averages their poses via SLERP, applies an exponential moving average
filter, and broadcasts the result as TF frames and `PoseWithCovarianceStamped` topics.

---

## Repository layout

```
convoy_traj/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── convoy_traj_params.yaml   # default node parameters
│   ├── tag_calibration.yaml      # inter-tag calibration (written by calibrate_tags.py)
│   └── ukf.yaml                  # placeholder (currently empty)
├── include/convoy_traj/          # public headers (currently empty)
├── launch/
│   ├── convoy_traj.launch.py             # standalone, inline args
│   ├── convoy_traj_yaml.launch.py        # standalone, loads YAML config files
│   ├── combined_convoy_system.launch.py  # full system (ZED + AprilTag + node)
│   └── combined_convoy_composable.launch.py
├── scripts/
│   └── calibrate_tags.py         # inter-tag calibration tool
└── src/
    └── convoy_traj_node.cpp      # sole C++ source file
```

---

## Build commands

All commands are run from the **workspace root** (`ros2_ws/`), not the package directory.

```bash
# Source ROS 2 (do once per shell)
source /opt/ros/humble/setup.bash

# Build this package only (recommended during development)
colcon build --packages-select convoy_traj

# Build with full compiler output
colcon build --packages-select convoy_traj --event-handlers console_direct+

# Build in Release mode
colcon build --packages-select convoy_traj --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace after building
source install/setup.bash
```

---

## Run commands

```bash
# Run the node directly
ros2 run convoy_traj convoy_traj_node

# Launch with inline parameter arguments
ros2 launch convoy_traj convoy_traj.launch.py alpha:=0.8 axes_offset_x:=1.5

# Launch loading YAML config
ros2 launch convoy_traj convoy_traj_yaml.launch.py

# Launch full system (ZED camera + AprilTag detector + convoy_traj node)
ros2 launch convoy_traj combined_convoy_system.launch.py

# Override a parameter at runtime
ros2 param set /convoy_traj_node alpha 0.5
```

---

## Test and lint commands

There are **no unit or integration tests** in this package. The CMake `BUILD_TESTING`
block runs `ament_lint_auto`; copyright and cpplint checks are explicitly disabled.

```bash
# Run ament linters (from workspace root)
colcon test --packages-select convoy_traj
colcon test-result --verbose

# Run a single ament linter manually (e.g., ament_cppcheck)
ament_cppcheck src/convoy_traj_node.cpp

# Check for unused includes / style (requires clang-tidy in PATH)
clang-tidy src/convoy_traj_node.cpp -- -std=c++17
```

To add a unit test, create `test/test_<name>.cpp`, register it in `CMakeLists.txt`
inside the `BUILD_TESTING` block with `ament_add_gtest`, and add `ament_cmake_gtest`
as a test dependency in `package.xml`.

---

## C++ code style

### Standard and warnings
- **C++17** (`cxx_std_17`). No C++20 features.
- Compiler flags: `-Wall -Wextra -Wpedantic`. All warnings must be resolved; do not
  suppress with pragmas or casts unless unavoidable, and always add a comment.

### Include order
Follow the pattern already used in `convoy_traj_node.cpp`:
1. ROS 2 core headers with double quotes: `"rclcpp/rclcpp.hpp"`
2. ROS 2 message/tf2 headers — double quotes for `tf2_ros/`, angle brackets for
   `tf2/` and `tf2_geometry_msgs/`
3. Blank line, then C++ standard library headers with angle brackets

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
```

### Naming conventions
| Element | Convention | Example |
|---|---|---|
| Classes | `PascalCase` | `ConvoyTrajNode` |
| Private member variables | `snake_case_` (trailing `_`) | `tf_buffer_`, `alpha_` |
| Methods and free functions | `snake_case` | `timer_callback()` |
| Local variables | `snake_case` | `averaged_transform`, `q_filtered` |
| ROS parameters | `snake_case` | `axes_offset_x`, `timer_period_ms` |
| Constants / macros | `SCREAMING_SNAKE_CASE` | `MAX_TIMEOUT_MS` |

### Class structure
- Inherit from `rclcpp::Node`; call `Node("node_name")` in the initializer list.
- Declare all parameters with `declare_parameter<T>(name, default)` before reading.
- Use **default member initialization** in the class body for simple scalars:
  `bool first_transform_{true};`, `double alpha_{0.1};`
- Use `std::make_unique` for `tf2_ros::Buffer`, `std::make_shared` for listeners,
  broadcasters, publishers, and timers.
- Keep `public` section first (constructor), then `private` (methods, then members).

### Error handling
- Wrap all `tf_buffer_->lookupTransform(...)` calls in `try/catch` blocks catching
  `tf2::TransformException`. On failure, log with `RCLCPP_WARN` and `return` early —
  **never** throw from a timer callback.
- Use `RCLCPP_INFO` for startup messages, `RCLCPP_WARN` for recoverable failures,
  `RCLCPP_ERROR` for non-recoverable failures. Avoid bare `std::cout`/`std::cerr`.

### Formatting
- 2-space indentation (matches existing source).
- Opening brace on the **same line** for functions, `if`, `for`, `try` blocks.
- No trailing whitespace. Keep lines under ~100 characters.

---

## Python launch file style

- Use `from X import Y` (no wildcard imports).
- Import `os` for path manipulation (`os.path.join`).
- Standard entry point: `def generate_launch_description() -> LaunchDescription:`.
- Name argument variables with the `_arg` suffix: `alpha_arg`, `axes_offset_x_arg`.
- Name node variables after their executable/role: `convoy_traj_node`, `zed_camera_node`.
- Always pass `output='screen'` to `Node(...)` for visibility during development.
- Group `DeclareLaunchArgument` objects first, then node definitions, then the return.
- Add a `#!/usr/bin/env python3` shebang and module-level docstring to new launch files.

---

## YAML config style

- Node parameters live under `<node_name>: ros__parameters:`.
- Use inline comments (`#`) to document units and valid ranges.
- Group parameters by category (frames, timing, filtering, covariance, geometry).

---

## Key topics and frames

| Name | Type | Direction | Description |
|---|---|---|---|
| `/tag_pose` | `PoseWithCovarianceStamped` | publish | Filtered pose of front vehicle |
| `/front_axes_pose` | `PoseWithCovarianceStamped` | publish | Pose of front axle offset point |
| `front_vehicle` | TF dynamic | broadcast | Camera → front vehicle frame |
| `front_axes` | TF static | broadcast | front_vehicle → front_axes offset |

Source frames: `zed_left_camera_optical_frame` → `tag36h11:0` / `tag36h11:1`

---

## Inter-tag calibration and single-tag fallback

The two AprilTags are mounted side-by-side on the same face of the front vehicle.
Their rigid relative pose (tag0 → tag1) is captured once by the calibration script
and stored in `config/tag_calibration.yaml`. The node then uses it to reconstruct
a missing tag's pose whenever only one is visible to the camera.

### Calibration workflow

```bash
# 1. Build and source the workspace at least once first
colcon build --packages-select convoy_traj && source install/setup.bash

# 2. Make sure the camera and AprilTag detector are already running so that
#    tag36h11:0 and tag36h11:1 TF frames are being published.
#    Both tags MUST be simultaneously visible to the camera.

# 3. Run the calibration script
ros2 run convoy_traj calibrate_tags.py \
    --output ~/ros2_ws/src/convoy_traj/config/tag_calibration.yaml \
    --samples 100

# 4. Rebuild to install the new calibration YAML to the share directory
colcon build --packages-select convoy_traj && source install/setup.bash

# 5. Launch the convoy_traj node — tag_calib_enabled: true is picked up automatically
ros2 launch convoy_traj convoy_traj_yaml.launch.py
```

Optional calibration flags:
- `--samples N` — number of frames to average (default: 100)
- `--timeout S` — max wall-clock seconds to wait for both tags (default: 60)
- `--tag0 FRAME` / `--tag1 FRAME` — override frame names if not using defaults

### How single-tag fallback works

| Tags visible | Behaviour |
|---|---|
| Both | Average as normal (unchanged from pre-calibration behaviour) |
| Only tag0 | Reconstruct tag1: `T_cam_tag1 = T_cam_tag0 ∘ T_tag0_tag1_calib` |
| Only tag1 | Reconstruct tag0: `T_cam_tag0 = T_cam_tag1 ∘ T_tag0_tag1_calib⁻¹` |
| Neither | `RCLCPP_WARN` + early return (no output published) |

When a tag is reconstructed a `RCLCPP_WARN` is emitted every cycle so it is visible
in logs. The EMA filter (`alpha_`) continues to operate on the averaged result
regardless of which path was taken.

### New parameters (from `tag_calibration.yaml`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `tag_calib_enabled` | `bool` | `false` | Enable single-tag fallback |
| `tag_calib_tx/ty/tz` | `double` | `0.0` | Translation of tag1 in tag0's frame (m) |
| `tag_calib_qx/qy/qz/qw` | `double` | `0/0/0/1` | Rotation of tag1's frame in tag0's frame |
| `tag_calib_samples` | `int` | `0` | Number of samples averaged (informational) |

---

## Common pitfalls

- `alpha_` in `convoy_traj_params.yaml` defaults to `0.8` (responsive); the in-code
  default is `0.1` (heavily smoothed). Always load the YAML or set the param explicitly.
- The `include/convoy_traj/` directory is empty. Place new headers there and update
  `target_include_directories` in `CMakeLists.txt` if needed.
- `config/ukf.yaml` is a placeholder — do not rely on it being populated.
- Both `convoy_traj_component` (shared lib) and `convoy_traj_node` (executable) are
  built from the same source file. Changes to `src/convoy_traj_node.cpp` affect both.
- `package.xml` maintainer and license fields are still placeholders (`TODO`). Update
  them before any public release.
- When `tag_calib_enabled: false` (uncalibrated), a single visible tag produces a
  `RCLCPP_WARN` and no output — it does **not** fall back to publishing the single tag.
- The calibration YAML is **overwritten** by `colcon build` if you edit the source
  `config/tag_calibration.yaml` — make sure to save calibration results to the source
  tree, not only to the install directory.
