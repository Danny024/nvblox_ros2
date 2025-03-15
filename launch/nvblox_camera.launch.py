from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nvblox_ros2',
            executable='nvblox_ros2_node',
            name='nvblox_ros2_node',
            output='screen',
            parameters=[
                {'voxel_size': 0.1},
                {'depth_topic': '/camera/depth/image_rect_raw'},
                {'color_topic': '/camera/color/image_raw'},
                {'pose_topic': '/camera/pose'},
                {'camera_info_topic': '/camera/depth/camera_info'},
                {'map_frame': 'map'},
                {'max_triangles': 10000}  # Added for performance
            ]
        )
    ])
