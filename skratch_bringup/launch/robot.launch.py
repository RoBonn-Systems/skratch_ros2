import xacro
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_args(context) -> list[LaunchDescriptionEntity]:

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        "robot_name",
        default_value=EnvironmentVariable("ROBOT_NAME"),
        description="Robot name, this is used as namespace to prefix all the nodes and topics"
    ))

    declared_args.append(DeclareLaunchArgument(
        "system",
        default_value="gazebo",
        description="Choose system to start, e.g. gazebo",
        choices=['gazebo', 'gz', 'none', 'real']
    ))

    # declared_args.append(DeclareLaunchArgument(
    #     "use_sim_time",
    #     default_value="false",
    #     description="Use sim time if true"
    # ))
    #
    # if LaunchConfiguration("system").perform(context) in ["gazebo", "gz"]:
    #     LogInfo(msg="Running simulation system, setting up use_sim_time to true.")

    declared_args.append(DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 automatically with this launch file."
    ))

    declared_args.append(DeclareLaunchArgument(
        "controllers_file",
        default_value="skratch_controllers.yml",
        description="file containing the definition of the controllers used by the controller_manager. Important: this file is expected to be in the directory `skratch_bringup/config/your_controllers.yml`."
    ))

    declared_args.append(DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="[NOT YET IMPLEMENTED] World file used by gazebot to start the simulation."
    ))

    declared_args.append(DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="x-coordinate to make the robot spawn in the simulator."
    ))

    declared_args.append(DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="y-coordinate to make the robot spawn in the simulator."
    ))

    declared_args.append(DeclareLaunchArgument(
        "z",
        default_value="0.5",
        description="z-coordinate to make the robot spawn in the simulator."
    ))

    return declared_args


def launch_setup(context) -> list[LaunchDescriptionEntity]:

    controllers_path_performed = PathJoinSubstitution([FindPackageShare("skratch_bringup"), "config", LaunchConfiguration("controllers_file")]).perform(context)

    robot_description_content = xacro.process_file(
        PathJoinSubstitution([FindPackageShare("skratch_description"), "robots", "robot.urdf.xacro"]).perform(context),
        mappings={
            # this mapping reads the simulator of choice and adds the `sim_` prefix,
            # therefor if the simulator is gazebo this returns `sim_gazebo`.
            # 
            # note that **this is mapped to the xacro arguments**,
            # assuming there is a `sim_gazebo` xacro arg defined this is set to true,
            # if the argument does not exist like `sim_something` nothing happens
            f"sim_{LaunchConfiguration('system').perform(context)}": "true",
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "controllers": controllers_path_performed,
            # "use_sim_time": LaunchConfiguration("use_sim_time").perform(context)
        }
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            # {"use_sim_time": LaunchConfiguration("use_sim_time")}, # TODO: Check if this is required
        ]
    )


    # controller_manager
    controller_manager = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "-p", controllers_path_performed,
            "joint_state_broadcaster", 
        ],
    )

    gz_spawn = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ]),
                launch_arguments={
                    "gz_args": ['-r ', LaunchConfiguration("world")], # `-r` start running simulation immediately
                    'on_exit_shutdown': 'True'
                }.items()
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', LaunchConfiguration("robot_name"),
                    '-topic', "robot_description",
                    '-x', LaunchConfiguration("x"),
                    '-y', LaunchConfiguration("y"),
                    '-z', LaunchConfiguration("z")
                ],
                output='both',
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "-p", controllers_path_performed,
                    "joint_state_broadcaster",  # ???? check if we need this
                ],
            )
        ],
        condition=LaunchConfigurationEquals("system", "gz")
    )

    gazebo_spawn_and_control = GroupAction(
        actions=[
            IncludeLaunchDescription(PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_entity",
                arguments=[
                    "-topic", "robot_description",
                    "-x", LaunchConfiguration("x"),
                    "-y", LaunchConfiguration("y"),
                    "-z", LaunchConfiguration("z"),
                    "-entity", LaunchConfiguration("robot_name")]
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "-p", controllers_path_performed,
                    "joint_state_broadcaster", 
                ],
            )
        ],
        condition=LaunchConfigurationEquals("system", "gazebo")
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
        PushRosNamespace(LaunchConfiguration("robot_name")), # this help to add namespace to all entities
        robot_state_publisher,
        gazebo_spawn_and_control,
        gz_spawn,
        rviz
    ]


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
