from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastbot_racing',
            executable='lidar_test.py',
            name='lidar_test',
            output='screen'
        )
    ])