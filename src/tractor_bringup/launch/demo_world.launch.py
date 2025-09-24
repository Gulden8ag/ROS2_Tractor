from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Flags básicas: puedes desactivar RViz o la GUI de joints si lo deseas
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gui  = LaunchConfiguration('use_gui',  default='true')

    desc_share = get_package_share_directory('tractor_description')
    display_launch = os.path.join(desc_share, 'launch', 'display.launch.py')

    # (Más adelante) Podremos agregar:
    # - world := nombre_del_world.sdf
    # - use_gazebo := true/false
    # - bridges, control, sensores, logging, etc.

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui',  default_value='true'),

        # Incluimos el display de la descripción y le pasamos el flag de GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch),
            launch_arguments={
                'use_gui': use_gui
            }.items()
        ),

        # Nota: el RViz se lanza dentro de tractor_description/display.launch.py.
        # Si quisieras poder apagar RViz desde acá, mueve el nodo RViz al bringup
        # y condiciona su arranque con use_rviz. Por ahora lo mantenemos simple.
    ])
