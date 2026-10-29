from setuptools import setup

package_name = 'tapmap'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/robot.launch.py', 'launch/robot_wifi.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/tapmap.rviz']),
        ('share/' + package_name + '/config', ['config/tapmap.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TapMap Team',
    maintainer_email='tapmap@example.com',
    description='TapMap full ROS2 stack (simulation + real ESP32 hardware)',
    license='MIT',
    entry_points={'console_scripts': [
        # simulation
        'sim_firmware = tapmap.sim_firmware:main',
        # real hardware
        'serial_bridge = tapmap.serial_bridge:main',
        'wifi_bridge = tapmap.wifi_bridge:main',
        'base_controller = tapmap.base_controller:main',
        'deadreckon_controller = tapmap.deadreckon_controller:main',
        # shared pipeline
        'audio_classifier = tapmap.audio_classifier:main',
        'defect_mapper = tapmap.defect_mapper:main',
        'coverage_planner = tapmap.coverage_planner:main',
        'report_generator = tapmap.report_generator:main',
    ]},
)
