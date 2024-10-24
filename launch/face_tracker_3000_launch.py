from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='face_tracker_3000',
            executable='face_offset',
            name='face_offset_node',
            output='screen'
        ),
        Node(
            package='face_tracker_3000',
            executable='aimbot',
            name='aimbot_node',
            output='screen'
        ),
    ])
