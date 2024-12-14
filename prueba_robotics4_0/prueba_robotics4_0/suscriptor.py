"""
@file test_integration.py
@Autor: Sofia Milagros Castaño Vanegas
@date: 2024-12-14
@version: 1.0
@brief Implementación de un test automatizado para validar la comunicación entre un nodo
ROS 2 publicador y un nodo suscriptor utilizando `pytest`.

Este archivo contiene la implementación de un test que valida la comunicación entre dos nodos 
de ROS 2: un publicador y un suscriptor. Se utiliza el framework `pytest` para manejar la 
ejecución de los tests y `rclpy` para la gestión de nodos y el sistema de logging de ROS 2.

@includes
- pytest: Framework para realizar tests automatizados.
- rclpy: Biblioteca de ROS 2 para manejar nodos y comunicación.
- rclpy.executors.SingleThreadedExecutor: Administrador de nodos de un solo hilo para ejecutar tareas.
- prueba_robotics4_0.publicador.Publisher: Nodo publicador para pruebas.
- prueba_robotics4_0.suscriptor.Suscriber: Nodo suscriptor para pruebas.

@clases
- Ninguna.

@metodos
- ros2_nodes: Fixture de pytest que configura los nodos y el ejecutor para los tests.
- test_pub_sub: Test que valida la comunicación entre los nodos publicador y suscriptor.

@miembros
- logger: Objeto `rclpy.logging.Logger` para manejar los logs del test.
"""
import pytest
import rclpy
from prueba_robotics4_0.publicador import Publisher  # Nodo publicador
from prueba_robotics4_0.suscriptor import Suscriber  # Nodo suscriptor
from rclpy.executors import SingleThreadedExecutor


# Configuración del sistema de logging de ROS 2
rclpy.init()  # Inicializa ROS 2
logger = rclpy.logging.get_logger('test_logger')  # Configura un logger para el test
rclpy.logging.set_logger_level(logger.name, rclpy.logging.LoggingSeverity.INFO)  # Define el nivel de logging


@pytest.fixture
def ros2_nodes():
    """
    @brief Fixture de pytest que configura los nodos y el ejecutor para los tests.
    Inicializa los nodos `Publisher` y `Suscriber`, así como un ejecutor de un solo hilo 
    para manejar la ejecución de estos nodos durante las pruebas.
    
    @return tuple: (Nodo `Publisher`, Nodo `Suscriber`, Ejecutor `SingleThreadedExecutor`).
    """
    talker_node = Publisher()
    listener_node = Suscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(talker_node)
    executor.add_node(listener_node)

    yield talker_node, listener_node, executor

    # Cleanup
    talker_node.destroy_node()
    listener_node.destroy_node()
    rclpy.shutdown()


def test_pub_sub(ros2_nodes):
    """
    @brief Test que valida la comunicación entre los nodos publicador y suscriptor.
    El test comprueba que el nodo suscriptor recibe mensajes enviados por el nodo publicador 
    dentro de un tiempo límite especificado.
    
    @param ros2_nodes Tuple retornado por el fixture que incluye los nodos y el ejecutor.
    """
    talker_node, listener_node, executor = ros2_nodes
    received_message = None
    timeout = 10  # Tiempo límite en segundos para recibir un mensaje
    timeout_limit = timeout * 10  # Límite en iteraciones (10 ciclos por segundo)

    # Log inicial del test
    logger.info("Starting test for valid message communication.")

    # Ejecuta el loop para intercambiar mensajes entre los nodos
    for i in range(timeout_limit):
        executor.spin_once(timeout_sec=1)
        if listener_node.received_msg:  # Verifica si el suscriptor recibió un mensaje
            received_message = listener_node.received_msg
            break

    # Asegura que se haya recibido un mensaje
    assert received_message is not None, f"Timeout after {timeout} seconds. No se recibieron mensajes."

    # Verifica el formato del mensaje recibido
    assert received_message.startswith("Hello Robotics 4.0:"), (
        f"Mensaje inesperado recibido: {received_message}. Se esperaba un mensaje que comience con 'Hello Robotics 4.0:'."
    )

    # Log del mensaje recibido
    logger.info(f"Received valid message: {received_message}")
