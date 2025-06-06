from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_integration')
    json_path_default = os.path.join(pkg_share, 'data/straight_json.json') #change to: curve_json.json
    pcap_path_default = os.path.join(pkg_share, 'data/straight_pcap.pcap') #change to: curve_pcap.pcap
    
    
    json_path = LaunchConfiguration('json_path')
    pcap_path = LaunchConfiguration('pcap_path')
    skip_time = LaunchConfiguration('skip_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'json_path',
            default_value=json_path_default,
            description='Path to the JSON metadata file'
        ),
        DeclareLaunchArgument(
            'pcap_path',
            default_value=pcap_path_default,
            description='Path to the PCAP file'
        ),
        DeclareLaunchArgument(
            'skip_time',
            ##############################################################
            default_value='18.0', # put the value of skip time
            ###################################################################
            description='Time in seconds to skip at the beginning of the PCAP'
        ),

        Node(
            package='lidar_integration',
            executable='total_reader',
            name='total_reader_node',
            output='screen',
            parameters=[
                {'json_path': json_path},
                {'pcap_path': pcap_path},
                {'skip_time': skip_time}
            ]
        ),

        Node(
            package='lidar_integration',
            executable='imu_reader',
            name='imu_node',
            output='screen',
            parameters=[
                {'json_path': json_path},
                {'pcap_path': pcap_path},
                {'skip_time': skip_time}
            ]
        ),

        Node(
            package='lidar_integration',
            executable='localization',
            name='localization_node',
            output='screen'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
        ),

        Node(
            package='lidar_integration',
            executable='crop_box_filter',
            name='crop_box_node',
            output='screen',
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),

        Node(
            package='lidar_integration',
            executable='cone_detection',
            name='cone_detection_node',
            output='screen',
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )

    ])
