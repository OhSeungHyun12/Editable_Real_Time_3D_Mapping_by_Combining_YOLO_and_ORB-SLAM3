from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라 노드 (웹캠)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen'
        ),

        # topic_tools를 이용한 FPS 제한 (15FPS)
        Node(
            package='topic_tools',
            executable='throttle',
            name='image_throttle',
            arguments=['messages', '/image_raw', '15', '/image_raw_low'],
            output='screen'
        ),

        # YOLO + ORB-SLAM 실행
        Node(
            package='yolo_orb3_ros2',
            executable='mono_ar',
            name='yolo_orb_slam',
            output='screen',
            arguments=[
                '/home/ruherpan/YOLO_ORB_SLAM3/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/home/ruherpan/YOLO_ORB_SLAM3/ORB_SLAM3/Examples/ROS2/webcam.yaml'
            ],
            remappings=[
                ('/image_raw', '/image_raw_low')
            ]
        )
    ])
