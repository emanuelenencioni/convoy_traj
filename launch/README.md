# Combined Convoy System Launch

This directory contains launch files to run the complete convoy tracking system with all three nodes:
1. **zed_camera_no_cuda** - ZED camera driver
2. **apriltag_ros** - AprilTag detection
3. **convoy_traj** - Convoy trajectory tracking

## Launch Files

### 1. `combined_convoy_system.launch.py` (Recommended)
Main launch file that runs all three nodes together in the same ROS2 process.

**Usage:**
```bash
ros2 launch convoy_traj combined_convoy_system.launch.py
```

**With custom parameters:**
```bash
ros2 launch convoy_traj combined_convoy_system.launch.py \
  video_device:=/dev/video0 \
  frame_width:=2560 \
  frame_height:=720 \
  apriltag_size:=0.12
```

### 2. `combined_convoy_composable.launch.py`
Alternative version prepared for composable nodes (currently uses regular nodes).

## Docker Usage

To run inside a Docker container:

```bash
# First, ensure your workspace is built
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash  # or your ROS2 distribution
colcon build --packages-select convoy_traj apriltag_ros zed_camera_no_cuda

# Source the workspace
source install/setup.bash

# Run the combined launch file
ros2 launch convoy_traj combined_convoy_system.launch.py
```

## Important Configuration

### Camera Topics
Make sure the topic remappings in the launch file match your ZED camera node's output:
- Default: `/zed_splitter_node/left/image_raw`
- Default: `/zed_splitter_node/left/camera_info`

If your ZED camera publishes to different topics, modify the remappings in the launch file.

### Video Device
The ZED camera might appear as:
- `/dev/video0`
- `/dev/video2`
- `/dev/video4`

Check available devices with:
```bash
ls -l /dev/video*
```

### Docker Considerations
If running in Docker, ensure:
1. Camera device is accessible in container (use `--device /dev/video2:/dev/video2`)
2. Display is forwarded for visualization (use `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`)
3. Proper permissions for video devices

Example Docker run command:
```bash
docker run -it --rm \
  --device /dev/video2:/dev/video2 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/ros2_ws:/workspace \
  your_ros2_image \
  bash
```

## Launch Parameters

### ZED Camera Parameters
- `zed_conf_file`: Path to camera calibration file (default: auto-detected)
- `video_device`: Video device path (default: `/dev/video2`)
- `frame_width`: Camera frame width (default: `2560`)
- `frame_height`: Camera frame height (default: `720`)
- `fps`: Frames per second (default: `30`)

### AprilTag Parameters
- `apriltag_size`: Size of AprilTags in meters (default: `0.12`)

### Convoy Trajectory Parameters
- `camera_frame_id`: Camera optical frame name (default: `zed_left_camera_optical_frame`)
- `tag0_frame_id`: First AprilTag frame (default: `tag36h11:0`)
- `tag1_frame_id`: Second AprilTag frame (default: `tag36h11:1`)
- `alpha`: EMA smoothing factor 0–1 (default: `0.1`, YAML default: `0.8`)
- `axes_offset_x/y/z`: Translation from `front_vehicle` to `front_axes` in metres (default: `0.0`)
- `tf_max_age_ms`: Max TF transform age in ms before tag is treated as gone (default: `1000`)

See the [top-level README](../README.md) for full parameter reference and the
inter-tag calibration workflow.

## Troubleshooting

### No camera detected
```bash
# Check if camera is recognized
v4l2-ctl --list-devices

# Check permissions
ls -l /dev/video*
```

### Topics not connecting
```bash
# List active topics
ros2 topic list

# Echo camera topics to verify output
ros2 topic echo /zed_splitter_node/left/image_raw --no-arr
ros2 topic echo /apriltag_node/detections
```

### Build errors
```bash
# Clean build
cd ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Node Communication Flow

```
ZED Camera Node
    ↓ (image_raw, camera_info)
AprilTag Node
    ↓ (detections, transforms)
Convoy Trajectory Node
    ↓ (averaged pose, transforms)
```
