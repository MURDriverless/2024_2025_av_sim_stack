from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. CSV → PointCloud2 publisher
        Node(
            package='csv_to_pointcloud_node',
            executable='simul_ready',
            name='simul_ready',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 2. CropBox filtering
        Node(
            package='csv_to_pointcloud_node',
            executable='crop_box_filter',
            name='crop_box_filter',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 3. Cone Detection
        Node(
            package='csv_to_pointcloud_node',
            executable='cone_detection',
            name='cone_detection',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # 4. Reflectivity & Signal-based Markers (remapped to cropped)
        Node(
            package='csv_to_pointcloud_node',
            executable='unified_marker_visualizer',
            name='unified_marker_visualizer',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 5. RViz2 Viewer (fixed frame must be set to 'lidar_frame')
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[]
        )
    ])
