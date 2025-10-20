from launch import LaunchDescription
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

    # Include the vehicle launch with a 10-second delay
    delayed_vehicle_launch = TimerAction(
        period=5.0,  # wait 10 seconds
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
        delayed_vehicle_launch
    ])
