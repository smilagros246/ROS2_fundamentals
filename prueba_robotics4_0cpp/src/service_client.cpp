/**
 * @file service_client.cpp
 * @author Sofia Mialgros Castaño Vanegas
 * @date 2024-12-14
 * @version 1.0
 * @brief Cliente ROS 2 que solicita el cálculo del cuadrado de un número.
 * 
 * Este archivo implementa un cliente para el servicio `SquareService` que permite al usuario ingresar un número
 * desde la consola y recibe el cuadrado de dicho número a través del servicio. El cliente continúa solicitando
 * números hasta que el usuario decide salir escribiendo "exit".
 * 
 * @includes
 * - rclcpp: Biblioteca de ROS 2 para manejar nodos y clientes.
 * - prueba_robotics4_0cpp/srv/square.hpp: Definición del servicio `Square`.
 * - memory: Manejo de punteros inteligentes en C++.
 * - cstdlib: Para conversión de cadenas a números y manejo de excepciones.
 * - iostream: Para entrada y salida de consola.
 * 
 * @functions
 * - main: Punto de entrada del cliente. Permite la interacción con el usuario y maneja las solicitudes al servicio.
 */

#include "rclcpp/rclcpp.hpp"
#include "prueba_robotics4_0cpp/srv/square.hpp"   

#include <memory>
#include <cstdlib>
#include <iostream>

using namespace std::chrono_literals;

/**
 * @brief Punto de entrada principal para el cliente del servicio "square".
 * 
 * Este cliente solicita números al usuario mediante la consola, envía solicitudes al servicio
 * para calcular el cuadrado y muestra la respuesta. Si el usuario escribe "exit", el cliente termina.
 * 
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  

  // Crear un nodo para el cliente
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("square_client");

  // Crear un cliente para el servicio "square"
  rclcpp::Client<prueba_robotics4_0cpp::srv::Square>::SharedPtr client =
    node->create_client<prueba_robotics4_0cpp::srv::Square>("square");

  // Esperar a que el servicio esté disponible
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrumpido mientras se espera el servicio. Saliendo.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servicio no disponible, esperando...");
  }

  int number = 0;  
  std::string input;  

  // Bucle para interactuar con el usuario
  while (rclcpp::ok()) {
    std::cout << "Ingrese un número para calcular su cuadrado (o escriba 'exit' para salir): ";
    std::getline(std::cin, input);

    
    if (input == "exit") {
      break;  
    }
    
    try {
      number = std::stoi(input);
    } catch (const std::invalid_argument& e) {
      std::cerr << "Entrada invalida, por favor ingrese un número válido." << std::endl;
      continue;  
    }

    // Crear la solicitud para el servicio
    auto request = std::make_shared<prueba_robotics4_0cpp::srv::Square::Request>();
    request->number = number;

    // Enviar la solicitud y manejar la respuesta
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "El cuadrado de %d es: %d", number, result.get()->square);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Fallo en llamar al servicio square");
    }
  }

  rclcpp::shutdown();  
  return 0;  
}
