from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mrg_modimoop_description')

    # Path to your xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'mrg_modimoop.urdf.xacro')

    # Declare rviz config argument (optional)
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(pkg_share, 'config', 'view.rviz'),
        description='Absolute path to rviz config file'
    )

    # Convert xacro -> urdf
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )


    return LaunchDescription([
        rviz_config_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}]
        ),

        # Joint State Publisher GUI (so you can move joints interactively)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        )
    ])
