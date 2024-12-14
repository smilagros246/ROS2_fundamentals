/**
 * @file service_server.cpp
 * @author Sofia Mialgros Castaño Vanegas
 * @date 2024-12-14
 * @version 1.0
 * @brief Implementación de un servicio ROS 2 "SquareService" que calcula el cuadrado de un número.
 * 
 * Este archivo contiene la implementación de un servidor de servicio llamado `SquareService` 
 * que responde a solicitudes de cálculo del cuadrado de un número entero proporcionado por un cliente.
 * 
 * @includes
 * - rclcpp: Biblioteca de ROS 2 para manejar nodos y servicios.
 * - prueba_robotics4_0cpp/srv/square.hpp: Definición del servicio `Square`.
 * - memory: Biblioteca estándar de C++ utilizada para manejar punteros inteligentes.
 * 
 * @functions
 * - compute_square: Función que realiza el cálculo del cuadrado del número recibido y envía la respuesta.
 * - main: Punto de entrada principal del programa que inicializa y ejecuta el servidor del servicio.
 */

#include "rclcpp/rclcpp.hpp"
#include "prueba_robotics4_0cpp/srv/square.hpp"

#include <memory>

/**
 * @brief Función de callback que procesa solicitudes para calcular el cuadrado de un número.
 * 
 * @param request Puntero compartido a la solicitud del cliente, que contiene el número de entrada.
 * @param response Puntero compartido a la respuesta, donde se almacena el resultado del cálculo.
 */
void compute_square(const std::shared_ptr<prueba_robotics4_0cpp::srv::Square::Request> request,
                    std::shared_ptr<prueba_robotics4_0cpp::srv::Square::Response> response)
{
  response->square = request->number * request->number;  // Calcular el cuadrado
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requerimiento recibido: numero: %d", request->number);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Respuesta enviada: cuadrado: %d", response->square);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // Inicializa la biblioteca rclcpp

  // Crear un nodo para el servicio "square"
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("square_service");

  // Crear el servicio "square"
  rclcpp::Service<prueba_robotics4_0cpp::srv::Square>::SharedPtr service =
    node->create_service<prueba_robotics4_0cpp::srv::Square>("square", &compute_square);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Listo para calcular cuadrado de numero.");

  rclcpp::spin(node);  

  rclcpp::shutdown();  
  return 0;  
}
