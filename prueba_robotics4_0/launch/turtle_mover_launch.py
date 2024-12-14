import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',  # Paquete de turtlesim
            executable='turtlesim_node',  # Nodo ejecutable
            name='turtlesim',  # Nombre del nodo
            output='screen'  # Salida en pantalla
        ),
        DeclareLaunchArgument(
            'speed', 
            default_value='1.0', 
            description='Velocidad de la tortuga'
        ),
        Node(
            package='prueba_robotics4_0',  
            executable='turtle_mover',  
            name='turtle_mover_node',
            output='screen',
            parameters=[{'speed': 1.0}]  
        )
    ])
