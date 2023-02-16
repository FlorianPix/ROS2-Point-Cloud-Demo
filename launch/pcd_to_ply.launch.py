import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'data/rosbag2_2023_02_08-15_23_22'],
            output='screen'
        ),
        Node(
            package='pcd_demo',
            executable='pcd_to_ply_node',
            name='pcd_to_ply'),
  ])