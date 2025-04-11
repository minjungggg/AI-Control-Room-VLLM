from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_ai_control',  # 여기에 실제 패키지 이름 입력
            executable='image_saver',     # image_saver.py의 실행 파일 이름
            name='image_saver_node'
        ),
        Node(
            package='ros_gz_ai_control',  # 동일한 패키지일 경우 그대로 유지
            executable='image_sub',       # image_sub.py의 실행 파일 이름
            name='image_sub_node'
        )])
