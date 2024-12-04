import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # v4l2_camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link_optical',
                'video_device': '/dev/video2',
            }]
        ),
        
        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__), '/ros-humble-dev-container/src/face_tracker_3000/rviz/rviz_face-tracker-3000.rviz'
            )]
        )
    ])
