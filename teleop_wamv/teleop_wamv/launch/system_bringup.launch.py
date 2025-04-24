from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_wamv',
            executable='image_saver',
            name='image_saver_node',
            output='screen'
        ),
        Node(
            package='teleop_wamv',
            executable='base64_pub',
            name='base64_pub_node',
            output='screen'
        ),
        Node(
            package='teleop_wamv',
            executable='gpt_description',
            name='gpt_description_node',
            output='screen'
        ),
        Node(
            package='teleop_wamv',
            executable='gpt_bridge',
            name='gpt_bridge_node',
            output='screen'
        )
    ])
