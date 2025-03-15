# nvblox_ros2

A ROS 2 wrapper for the `nvblox` library from NVIDIA-ISAAC-ROS, sourced from `https://github.com/NVIDIA-ISAAC-ROS/nvblox.git` as a submodule for GPU-accelerated 3D mapping.

## Prerequisites
- ROS 2 Humble (Ubuntu 22.04)
- CUDA Toolkit (e.g., 11.8) and NVIDIA GPU
- ROS 2 dependencies: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `nav_msgs`, `tf2_ros`, `message_filters`
- `libeigen3-dev`

## Installation
1. **Create Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   




#### `topics.md`
```markdown
# Topics Subscribed and Published by nvblox_ros2

## Subscribed Topics
1. **Depth Image** (`sensor_msgs/Image`)
   - **Default**: `/camera/depth/image_rect_raw`
   - **Encoding**: `32FC1`
   - **Parameter**: `depth_topic`
   - **Description**: Depth data in meters for camera-based mapping.
2. **Color Image** (`sensor_msgs/Image`)
   - **Default**: `/camera/color/image_raw`
   - **Encoding**: `rgb8`
   - **Parameter**: `color_topic`
   - **Description**: Optional RGB data for mesh coloring.
3. **Camera Pose** (`geometry_msgs/PoseStamped`)
   - **Default**: `/camera/pose`
   - **Parameter**: `pose_topic`
   - **Description**: Camera pose in `map_frame`.
4. **Camera Info** (`sensor_msgs/CameraInfo`)
   - **Default**: `/camera/depth/camera_info`
   - **Parameter**: `camera_info_topic`
   - **Description**: Camera intrinsics, required once.
5. **LiDAR Point Cloud** (`sensor_msgs/PointCloud2`)
   - **Default**: `/lidar/points`
   - **Parameter**: `lidar_topic`
   - **Description**: 3D points (x, y, z) for LiDAR mapping.

## Published Topics
1. **3D Mesh** (`visualization_msgs/Marker`)
   - **Topic**: `nvblox/mesh`
   - **Frame**: `map_frame` (default: `map`)
   - **Description**: Triangular mesh with per-vertex colors if available.
2. **2D Costmap** (`nav_msgs/OccupancyGrid`)
   - **Topic**: `nvblox/costmap`
   - **Frame**: `map_frame` (default: `map`)
   - **Description**: ESDF slice at z=0 (0=free, 50=near, 100=occupied).
