from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Load the SDF file from "description" package
    base_path = '/Users/taey/vllm_control_ws/src/AI-Control-Room-VLLM/gz-waves-models/models/wamv_camera'
    sdf_file = os.path.join(base_path, 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # For publishing and controlling the robot pose, we need joint states of the robot
    # Configure the robot model by adjusting the joint angles using the GUI slider
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[sdf_file],
        output=['screen']
    )
    
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       condition=IfCondition(LaunchConfiguration('rviz'))
    )


### 마지막 return 에서 위의 3개 노드 추가

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        joint_state_publisher,
        robot_state_publisher,
        rviz            
    ])