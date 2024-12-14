"""
@file publicador.py
@Autor: Sofia Milagros Castaño Vanegas
@date: 2024-12-07
@version: 1.0
@brief Implementación de un nodo ROS 2 "PublisherNode" que publica mensajes en el tópico "msg_send".
Este archivo contiene la implementación de un nodo llamado `Publisher` que publica mensajes periódicamente 
en el tópico "msg_send" utilizando mensajes de tipo `String`. Los mensajes incluyen un contador para 
demostrar la funcionalidad de publicación periódica.

@includes
- rclpy: Biblioteca de ROS 2 para manejar nodos y comunicación.
- std_msgs.msg.String: Tipo de mensaje `String` utilizado para la publicación de datos en el tópico.
- time: Biblioteca estándar de Python utilizada para operaciones relacionadas con tiempo.

@clase Publisher
@brief Nodo ROS 2 que publica mensajes periódicamente en el tópico "msg_send".
@metodos
- __init__: Constructor que inicializa el nodo `publisher_node` y configura el publicador y el temporizador.
- publish_message: Método que crea y publica mensajes en el tópico "msg_send". 
  También incrementa un contador para rastrear los mensajes enviados.

@miembros
- publisher_: Objeto `Publisher` que se encarga de publicar los mensajes en el tópico "msg_send".
- timer: Temporizador que controla la periodicidad de las publicaciones.
- count: Contador que almacena el número de mensajes publicados.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Publisher(Node):
    """
    @clase Publisher
    @brief Nodo ROS 2 que publica mensajes en el tópico "msg_send".
    """

    def __init__(self):
        """
        @brief Constructor que inicializa el nodo "publisher_node" y configura el publicador y el temporizador.
        """
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'msg_send', 10)  
        self.timer = self.create_timer(1.0, self.publish_message)  # Temporizador para publicaciones periódicas
        self.count = 0  

    def publish_message(self):
        """
        @brief Método que crea y publica mensajes en el tópico "msg_send".
        También incrementa un contador para rastrear los mensajes enviados.
        """
        msg = String()
        msg.data = f"Hello Robotics 4.0: {self.count}"  
        self.publisher_.publish(msg)  
        self.get_logger().info(f"Publishing: '{msg.data}'")  # Registro en los logs de ROS 2
        self.count += 1  # Incrementa el contador

def main(args=None):
    """
    @brief Punto de entrada principal del nodo ROS 2. Inicializa y ejecuta el nodo Publisher.
    """
    rclpy.init(args=args)
    node = Publisher()
    try:
        rclpy.spin(node)  # Ejecuta el nodo Publisher
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Destruye el nodo al finalizar
        rclpy.shutdown()  # Finaliza ROS 2

if __name__ == '__main__':
    main()


