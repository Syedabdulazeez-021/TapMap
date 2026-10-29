from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tapmap', executable='sim_firmware', output='screen'),
        Node(package='tapmap', executable='audio_classifier', output='screen'),
        Node(package='tapmap', executable='defect_mapper', output='screen'),
        Node(package='tapmap', executable='coverage_planner', output='screen'),
        Node(package='tapmap', executable='report_generator', output='screen'),
        # map frame == odom frame at scan start
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','map','odom']),
    ])
