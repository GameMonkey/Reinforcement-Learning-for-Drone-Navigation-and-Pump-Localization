import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 
                                                                                             'launch'),
                                                                                             '/online_async_launch.py']))

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock']
    )

    depth_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        remappings=[('/depth_camera/points','/cloud')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'   
        ),

        clock_bridge,
        depth_camera_bridge,
        slam_toolbox_node,

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  '/cloud'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.02,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'concurrency_level':0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
