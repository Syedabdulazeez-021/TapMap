"""Real-robot launch: ESP32 over serial instead of the simulator.

Brings up serial_bridge + base_controller in place of sim_firmware; the
planner / classifier / mapper / report nodes are identical to the sim.
Loads calibration from config/tapmap.yaml.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tapmap'), 'config', 'tapmap.yaml')
    return LaunchDescription([
        Node(package='tapmap', executable='serial_bridge', output='screen', parameters=[cfg]),
        # 4-wheel chassis has NO encoders -> dead-reckoning odom instead of base_controller
        Node(package='tapmap', executable='deadreckon_controller', output='screen', parameters=[cfg]),
        Node(package='tapmap', executable='audio_classifier', output='screen', parameters=[cfg]),
        Node(package='tapmap', executable='defect_mapper', output='screen'),
        Node(package='tapmap', executable='coverage_planner', output='screen'),
        Node(package='tapmap', executable='report_generator', output='screen'),

        # static frames
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.046', '0', '0', '0', '0', '0', 'base_link', 'tapper_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
    ])
