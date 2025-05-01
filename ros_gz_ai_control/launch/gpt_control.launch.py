from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_ai_control',
            executable='thrust_control',
            name='robot_motion_controller',
            output='screen'
        ),
        Node(
            package='ros_gz_ai_control',
            executable='gpt_commander',
            name='gpt_image_robot_controller',
            output='screen'
        ),
        Node(
            package='ros_gz_ai_control',
            executable='distance_calculator',
            name='distance_calculator_node',
            output='screen'
        ),
        Node(
            package='ros_gz_ai_control',
            executable='imu_velocity',
            name='velocity_calculator_node',
            output='screen'
        )
    ])
