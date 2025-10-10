# demo_world.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Flags básicas: puedes desactivar RViz o la GUI de joints si lo deseas
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gui  = LaunchConfiguration('use_gui',  default='true')
    world    = LaunchConfiguration('world',    default='my_world.sdf')
    robot_name = LaunchConfiguration('robot_name', default='mi_robot')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.05')

    # Paths
    desc_share = get_package_share_directory('tractor_description')
    display_launch = os.path.join(desc_share, 'launch', 'display.launch.py')
    world_path = PathJoinSubstitution([
        FindPackageShare('tractor_bringup'),
        'worlds',
        world
    ])

    # 1) Start Gazebo Harmonic
    gz_proc = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', world_path],
        output='screen'
    )

    # 2) Publish /robot_description (robot_state_publisher etc.)
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch),
        launch_arguments={
            'use_gui': use_gui,
            'use_rviz': use_rviz
        }.items()
    )

    # 3) Spawn into Gazebo from /robot_description (delay ensures param is ready)
    spawn = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', robot_name,
                    '-topic', '/robot_description',
                    '-x', x, '-y', y, '-z', z,
                    '-allow_renaming', 'true'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui',  default_value='true'),
        DeclareLaunchArgument('world',    default_value='my_world.sdf'),
        DeclareLaunchArgument('robot_name', default_value='mi_robot'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        gz_proc,
        display,
        spawn
    
    ])
