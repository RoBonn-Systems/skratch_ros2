import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    description_path = os.path.join(get_package_share_directory('skratch_description'))
    skratch_path = os.path.join(get_package_share_directory('skratch_bringup'))
    xacro_file = os.path.join(description_path, 'robots', 'robot.urdf.xacro')
    belt_description_config = xacro.process_file(xacro_file)
    rviz_config_file = os.path.join(skratch_path, 'config', 'robot.rviz')
    
    params = {'robot_description': belt_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])



