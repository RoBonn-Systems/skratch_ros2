import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_args(context):

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated time"
    ))


    return declared_args


def launch_setup(context):

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_description_content = xacro.process_file(
        PathJoinSubstitution([
            FindPackageShare("skratch_description"),
            "robots",
            "robot.urdf.xacro"
        ]).perform(context),
        mappings={
            # "use_sim_time": LaunchConfiguration("use_sim_time").perform(context)
        }
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return [
        joint_state_publisher,
        robot_state_publisher
    ]

def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
