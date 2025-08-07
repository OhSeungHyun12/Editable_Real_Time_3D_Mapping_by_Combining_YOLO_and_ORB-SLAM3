from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='throttle',
            name='topic_1',
            arguments=['messages', 'camera/color/image_raw', '15', 'camera/color/image_raw_low'],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='throttle',
            name='topic_2',
            arguments=['messages', 'camera/aligned_depth_to_color/image_raw', '15', 'camera/aligned_depth_to_color/image_raw_low'],
            output='screen'
        )
    ])
