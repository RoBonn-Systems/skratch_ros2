from launch import LaunchDescription, LaunchDescriptionEntity
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_args(context) -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim time if true"
    ))

    declared_args.append(DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 automatically with this launch file."
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    robot_description = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("skratch_bringup"),
            "launch",
            "states.launch.py"
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("skratch_bringup"),
        "config",
        "robot.rviz"
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration("start_rviz"))
    )

    return [
        PushRosNamespace(EnvironmentVariable("ROBOT_NAME")), # this help to add namespace to all entities
        robot_description,
        rviz
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
