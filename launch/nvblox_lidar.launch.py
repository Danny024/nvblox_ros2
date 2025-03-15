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
                {'voxel_size': 0.05},
                {'lidar_topic': '/lidar/points'},
                {'map_frame': 'map'},
                {'max_triangles': 10000}  # Added for performance
            ]
        )
    ])
