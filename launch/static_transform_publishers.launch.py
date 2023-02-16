import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '1.0', '6.123233995736766e-17',
                 'base_link', 'base'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '1.0', '6.123233995736766e-17',
                 'base_link', 'base_link_inertia'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.4999999999999999', '0.5', '0.5', '-0.5000000000000001',
                 'wrist_3_link', 'flange'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.5', '0.4999999999999999', '0.5', '-0.5000000000000001',
                 'flange', 'tool0'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0.0', '0.0', '0.0', '2.36', '-1.57', '0.0',
                 'tool0', 'camera_mount'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'camera_mount', 'zed2_base_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.015',
                 '0.0', '0.024997395914712332', '0.0', '0.9996875162757025',
                 'zed2_base_link', 'zed2_camera_center'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.06', '0.015',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_camera_center', 'zed2_left_camera_frame'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '-0.01', '0.0', '0.0',
                 '0.5', '-0.4999999999999999', '0.5', '-0.5000000000000001',
                 'zed2_left_camera_frame', 'zed2_left_camera_optical_frame'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_left_camera_frame', 'zed2_temp_left_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_left_camera_frame', 'zed2_temp_left_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_camera_center', 'zed2_baro_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_camera_center', 'zed2_mag_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_camera_center', 'zed2_mag_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '-0.06', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_camera_center', 'zed2_right_camera_frame'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '-0.01', '0.0', '0.0',
                 '0.5', '-0.4999999999999999', '0.5', '-0.5000000000000001',
                 'zed2_right_camera_frame', 'zed2_right_camera_optical_frame'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_right_camera_frame', 'zed2_temp_right_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.0', '0.0', '0.0',
                 '0.0', '0.0', '0.0', '1.0',
                 'zed2_left_camera_frame', 'zed2i_left_camera_frame'],
            output='screen'
        ),
    ])
