# nvblox_ros2

## Overview

`nvblox_ros2` is a ROS 2 wrapper for the `nvblox` library from NVIDIA-ISAAC-ROS, integrated as a Git submodule from `https://github.com/NVIDIA-ISAAC-ROS/nvblox.git`. This package leverages GPU acceleration to generate a 3D triangular mesh from camera depth images, designed for ROS 2 Humble on Ubuntu 22.04. It uses the `nvblox` library’s TSDF (Truncated Signed Distance Field) capabilities to process depth data, publishing the resulting mesh for visualization in RViz. This version is optimized for camera input, with LiDAR support removed due to compatibility issues in the current `nvblox` submodule.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2 Distribution**: Humble
- **CUDA Toolkit**: Version 11.8 or compatible, with an NVIDIA GPU
- **ROS 2 Packages**:
  - `rclcpp`
  - `sensor_msgs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `nav_msgs`
  - `tf2_ros`
  - `message_filters`
- **Additional Libraries**: `libeigen3-dev`

## Installation

1. **Create a ROS 2 Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   git clone <repository_url> src/nvblox_ros2
   cd src/nvblox_ros2
   git submodule update --init --recursive
   sudo apt update
   sudo apt install -y ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-geometry-msgs \
                    ros-humble-visualization-msgs ros-humble-nav-msgs ros-humble-tf2-ros \
                    ros-humble-message-filters libeigen3-dev
   cd ~/ros2_ws
   colcon build --packages-select nvblox_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```
## Usage

   ros2 launch nvblox_ros2 nvblox_camera.launch.py
   ros2 run rviz2 rviz2
   
## Configuration Parameters

**voxel_size** 
```(default: 0.1): Voxel size in meters for the TSDF grid.```

**depth_topic**  
```(default: **/camera/depth/image_rect_raw**): Source of depth images (32FC1 encoding).```

**color_topic** 
```(default: **/camera/color/image_raw**): Optional source of color images (rgb8 encoding).```

**pose_topic** 
(default: /camera/pose): Camera pose data.

**camera_info_topic** 
```(default: **/camera/depth/camera_info**): Camera intrinsics data.```

**map_frame** 
```(default: **map**): Reference frame for mesh and costmap output.```

**max_triangles** 
```(default: 10000): Maximum number of triangles in the mesh.```


## Subscribed Topics

**Depth Image** (sensor_msgs/Image):
```
Default: /camera/depth/image_rect_raw
Encoding: 32FC1
Parameter: depth_topic
Description: Depth data in meters for 3D mapping.
```
**Color Image** (sensor_msgs/Image):
```
Default: /camera/color/image_raw
Encoding: rgb8
Parameter: color_topic
Description: Optional RGB data for mesh coloring.
```

**Camera Pose** (geometry_msgs/PoseStamped):
```
Default: /camera/pose
Parameter: pose_topic
Description: Camera pose in the map_frame.
```

**Camera Info** (sensor_msgs/CameraInfo):
```
Default: /camera/depth/camera_info
Parameter: camera_info_topic
Description: Camera intrinsics, required once for initialization.
```

### Published Topics
`
**3D Mesh** (visualization_msgs/Marker):
```
Topic: /nvblox/mesh
Frame: map_frame (default: map)
Description: Triangular mesh with per-vertex colors if color data is provided.
```

**2D Costmap** (nav_msgs/OccupancyGrid):
```
Topic: /nvblox/costmap
Frame: map_frame (default: map)
Description: Placeholder ESDF slice at z=0 (all zeros) due to missing esdf_layer.h.
```

