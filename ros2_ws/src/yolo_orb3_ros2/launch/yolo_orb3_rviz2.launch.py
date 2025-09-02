import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Pakage dir
    # ==============================================================================

    orb_slam3_root_dir = os.getenv('ORB_SLAM3_ROOT')
    if orb_slam3_root_dir is None:
        raise Exception('Environment variable ORB_SLAM3_ROOT is not set!')
    
    yolo_orb3_ros2_share_dir = get_package_share_directory('yolo_orb3_ros2')

    # ==============================================================================

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen'
    )

    # Limit FPS: 30FPS
    image_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle',
        arguments=['messages', '/image_raw', '30', '/image_raw_low'],
        output='screen'
    )

    # Running OLO + ORB-SLAM
    yolo_orb_slam_node = Node(
        package='yolo_orb3_ros2',
        executable='mono_ar',
        name='yolo_orb_slam',
        output='screen',
        arguments=[
            os.path.join(orb_slam3_root_dir, 'Vocabulary', 'ORBvoc.txt'),
            os.path.join(yolo_orb3_ros2_share_dir, 'config', 'webcam.yaml')
        ],
        remappings=[
            ('/image_raw', '/image_raw_low') 
        ]
    )

    # Running Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(yolo_orb3_ros2_share_dir, 'rviz', 'slam.rviz')],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        image_throttle_node,
        yolo_orb_slam_node,
        rviz_node
    ])
