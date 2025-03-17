from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot = LaunchConfiguration("robot")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    use_ned_frame = LaunchConfiguration("use_ned_frame")
    
    world_name = LaunchConfiguration("world").perform(context)
    world_filename = f"{world_name}.world"
    world_filepath = PathJoinSubstitution(
        [FindPackageShare("waves_models"), "worlds", world_filename]
    )
    gz_args = [world_filepath]

    gz_args.append(" -s -r -v4")

    # Gazebo server launch with world file
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("gz_args", gz_args),
        ],
    )

    # Gazebo gui launch
    gui_config_filepath = PathJoinSubstitution(
        [FindPackageShare("ros_gz_ai_control"), "gui.config"]
    )
    gz_args_substitution = [
        TextSubstitution(text="-g --gui-config "),
        gui_config_filepath
    ]
    gz_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments=[
            ("gz_args", gz_args_substitution),
        ],
    )

    # Include robot launch with model name
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_ai_control"),
                        "upload_robot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": robot,
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "use_ned_frame": use_ned_frame,
        }.items(),
    )

    include = [gz_sim_launch, robot_launch, gz_gui_launch]

    return include


def generate_launch_description():
    # Kill any existing gzserver instances
    kill_gazebo = ExecuteProcess(
        cmd=["bash", "-c", "ps -ef | grep 'gz sim' | grep -v grep | awk '{print $2}' | xargs kill -9"],
        shell=False,
        output="screen",
    )

    # Declare the launch arguments with default values
    args = [
        DeclareLaunchArgument(
            "paused",
            default_value="true",
            description="Start the simulation paused",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Flag to enable the gazebo debug flag",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Flag to enable the gazebo headless mode",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="0",
            description="Adjust level of console verbosity",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="waves.world",
            description="Gazebo world file to launch",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="-18.0",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Initial z position",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Initial roll angle",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Initial pitch angle",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial yaw angle",
        ),
        DeclareLaunchArgument(
            "use_ned_frame",
            default_value="false",
            description="Flag to indicate whether to use the north-east-down frame",
        ),
    ]

    return LaunchDescription(
        [kill_gazebo] + args + [OpaqueFunction(function=launch_setup)])
