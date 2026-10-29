"""Real-robot launch over WiFi: wifi_bridge instead of serial_bridge.

Identical to robot.launch.py except the ESP32 link is a TCP socket
(wifi_bridge) rather than USB serial (serial_bridge). Everything else
- deadreckon_controller, classifier, mapper, planner, report - is the same.
Set the ESP32's IP in config/tapmap.yaml -> wifi_bridge: esp32_ip.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tapmap'), 'config', 'tapmap.yaml')
    return LaunchDescription([
        Node(package='tapmap', executable='wifi_bridge', output='screen', parameters=[cfg]),
        Node(package='tapmap', executable='deadreckon_controller', output='screen', parameters=[cfg]),
        Node(package='tapmap', executable='audio_classifier', output='screen', parameters=[cfg]),
        Node(package='tapmap', executable='defect_mapper', output='screen'),
        Node(package='tapmap', executable='coverage_planner', output='screen'),
        Node(package='tapmap', executable='report_generator', output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.046', '0', '0', '0', '0', '0', 'base_link', 'tapper_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
    ])
