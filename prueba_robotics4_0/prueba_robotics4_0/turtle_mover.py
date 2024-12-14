"""
@file turtle_mover.py
@Autor: Sofia Milagros Castaño Vanegas
@date: 2024-12-14
@version: 1.0
@brief Implementación de un nodo ROS 2 que controla el movimiento de una tortuga en el simulador TurtleSim.
Este archivo contiene la implementación de un nodo llamado `TurtleMover` que controla el movimiento 
de una tortuga utilizando el parámetro de velocidad `speed` y publicando mensajes de tipo `Twist` en el tópico 
`/turtle1/cmd_vel` para modificar la velocidad lineal de la tortuga.

@includes
- rclpy: Biblioteca de ROS 2 para manejar nodos y comunicación.
- geometry_msgs.msg.Twist: Tipo de mensaje `Twist` utilizado para controlar el movimiento de la tortuga.

@clase TurtleMover
@brief Nodo ROS 2 que controla el movimiento de una tortuga en el simulador TurtleSim.
@metodos
- __init__: Constructor que inicializa el nodo `turtle_mover`, declara el parámetro `speed`, 
  crea un publisher para el tópico `/turtle1/cmd_vel` y establece un timer para mover la tortuga.
- move_turtle: Método que publica el mensaje de velocidad en el tópico para mover la tortuga.
  
@miembros
- publisher_: Objeto `Publisher` que publica mensajes de tipo `Twist` en el tópico `/turtle1/cmd_vel`.
- timer: Objeto `Timer` que ejecuta el método `move_turtle` a intervalos regulares (0.1 segundos).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    """
    @clase TurtleMover
    @brief Nodo ROS 2 que controla el movimiento de una tortuga en el simulador TurtleSim.
    """

    def __init__(self):
        """
        @brief Constructor que inicializa el nodo `turtle_mover`, declara el parámetro `speed`, 
        crea un publisher para el tópico `/turtle1/cmd_vel` y establece un timer para mover la tortuga.
        """
        super().__init__('turtle_mover')  # Inicializa el nodo con el nombre 'turtle_mover'

        # Declarar el parámetro 'speed' con valor por defecto de 1.0
        self.declare_parameter('speed', 1.0)

        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        
        self.timer = self.create_timer(0.1, self.move_turtle)

    def move_turtle(self):
        """
        @brief Método que publica el mensaje de velocidad para mover la tortuga.
        Obtiene el valor del parámetro `speed` y lo utiliza para controlar la velocidad 
        lineal de la tortuga publicando un mensaje en el tópico `/turtle1/cmd_vel`.

        @details Este método se ejecuta a intervalos regulares, gracias al `timer` configurado en el constructor.
        """
   
        speed = self.get_parameter('speed').get_parameter_value().double_value

        
        msg = Twist()
        msg.linear.x = speed  # Controlar la velocidad lineal con el parámetro 'speed'

        # Publicar el mensaje
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando velocidad: {speed}')


def main(args=None):
    """
    @brief Punto de entrada principal del nodo ROS 2. Inicializa y ejecuta el nodo `TurtleMover`.
    """
    rclpy.init(args=args)

    turtle_mover = TurtleMover()

    # Ejecutar el nodo
    rclpy.spin(turtle_mover)

    # Finalizar la ejecución de ROS 2
    turtle_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
