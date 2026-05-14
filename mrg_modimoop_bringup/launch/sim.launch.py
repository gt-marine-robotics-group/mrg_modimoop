from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_launch_file = os.path.join(
        get_package_share_directory('lyoko_gz_bringup'),
        'launch',
        'spawn_world.launch.py'
    )

    vehicle_launch_file = os.path.join(
        get_package_share_directory('lyoko_gz_bringup'),
        'launch',
        'spawn_robot.launch.py'
    )

    # Include the world launch immediately
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_file),
        launch_arguments={'world': 'ocean'}.items()
    )

    delayed_vehicle_launch = TimerAction(
        period=1.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch_file),
                launch_arguments={
                    'vehicle_type': 'mrg_modimoop',
                    'world_name': 'ocean'}.items()
            )
        ]
    )

    return LaunchDescription([
        world_launch,
        delayed_vehicle_launch,

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_magnetic_field_msg': True,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True,
            }],
            remappings=[
                ('/imu/data_raw', '/modimoop/imu_raw'),
                ('/imu/mag', '/modimoop/mag'),
                ('/imu/data', '/modimoop/imu'),
            ]
        ),
    ])
