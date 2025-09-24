from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('tractor_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tractor.urdf.xacro')

    # Convierte Xacro a string URDF
    robot_description_config = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        # Opcional: ancla un frame fijo si en RViz usas "map" como Fixed Frame.
        # Si tu Fixed Frame es 'base_link', puedes omitir este nodo.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            name='map_to_base_link'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
            name='rviz2'
        ),
    ])
