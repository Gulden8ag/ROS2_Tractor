#display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gui  = LaunchConfiguration('use_gui',  default='true')
    use_map_static_tf = LaunchConfiguration('use_map_static_tf', default='false')

    pkg_share = get_package_share_directory('tractor_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tractor.urdf.xacro')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'display.rviz')  # tu archivo actual

    # Convierte Xacro a string URDF
    robot_description_config = xacro.process_file(urdf_path).toxml()
    robot_description = {'robot_description': robot_description_config}

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui',  default_value='true'),
        # Pon esto en true solo si en RViz tu "Fixed Frame" es 'map'
        DeclareLaunchArgument('use_map_static_tf', default_value='false'),

        # Solo si de verdad usas 'map' como Fixed Frame:
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            name='map_to_base_link',
            condition=IfCondition(use_map_static_tf)
        ),

        # Sliders para probar joints en RViz (no los necesita Gazebo)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui)
        ),

        # Publica TF desde la URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),

        # RViz con tu config (ahí puedes activar "Collision Enabled")
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            name='rviz2',
            condition=IfCondition(use_rviz)
        ),
    ])
