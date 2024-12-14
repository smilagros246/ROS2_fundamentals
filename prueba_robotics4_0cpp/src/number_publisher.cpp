/**
 * @file number_publisher.cpp
 * @author Sofia Mialgros Castaño Vanegas
 * @date 2024-12-14
 * @version 1.0
 * @brief Implementación de un nodo ROS 2 "NumberPublisher" que publica números aleatorios en el tópico "/number_stream".
 * 
 * Este archivo contiene la implementación de un nodo llamado `NumberPublisher` que publica números enteros 
 * aleatorios en el rango de 0 a 100 cada segundo. Los mensajes se envían utilizando el tipo de mensaje 
 * `std_msgs::msg::Int32`.
 * 
 * @includes
 * - rclcpp: Biblioteca de ROS 2 para manejar nodos y comunicación.
 * - std_msgs/msg/int32.hpp: Tipo de mensaje `Int32` utilizado para la publicación de números.
 * - cstdlib: Biblioteca estándar de C++ utilizada para generar números aleatorios.
 * - ctime: Biblioteca estándar de C++ para inicializar la semilla de generación de números aleatorios.
 * 
 * @class NumberPublisher
 * @brief Nodo ROS 2 que publica números aleatorios en el tópico "/number_stream".
 * 
 * @methods
 * - NumberPublisher: Constructor que inicializa el nodo, configura el publicador y el temporizador, 
 *   y establece la semilla para números aleatorios.
 * - publish_random_number: Método privado que genera un número aleatorio y lo publica en el tópico "/number_stream".
 * 
 * @members
 * - publisher_: Objeto `Publisher` que se encarga de publicar mensajes en el tópico "/number_stream".
 * - timer_: Temporizador que controla la periodicidad de las publicaciones.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cstdlib>
#include <ctime>

/**
 * @class NumberPublisher
 * @brief Nodo ROS 2 que publica números aleatorios en el rango 0-100 en el tópico "/number_stream".
 */
class NumberPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor que inicializa el nodo "number_publisher", configura el publicador y el temporizador.
     * 
     * El constructor inicializa la semilla para generar números aleatorios y crea un publicador 
     * para el tópico "/number_stream". También configura un temporizador que llama al método 
     * `publish_random_number` cada segundo.
     */
    NumberPublisher()
    : Node("number_publisher")
    {
        
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/number_stream", 10);
        
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberPublisher::publish_random_number, this));
        
        RCLCPP_INFO(this->get_logger(), "Number Publisher has started.");
        
        // Inicializa para generación de números aleatorios
        std::srand(std::time(nullptr));
    }

private:
    /**
     * @brief Método privado que genera un número aleatorio y lo publica en el tópico "/number_stream".
     * 
     * Este método crea un mensaje de tipo `std_msgs::msg::Int32` con un número aleatorio 
     * en el rango 0-100 y lo publica utilizando el publicador configurado.
     */
    void publish_random_number()
    {
        auto message = std_msgs::msg::Int32();
        message.data = std::rand() % 101;  // Genera un número aleatorio entre 0 y 100
        publisher_->publish(message);  
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);  
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_; /**< Publicador del tópico "/number_stream". */
    rclcpp::TimerBase::SharedPtr timer_; /**< Temporizador que controla la periodicidad de las publicaciones. */
};

/**
 * @brief Función principal que inicializa el nodo "NumberPublisher" y lo ejecuta.
 * 
 * La función principal inicializa la biblioteca ROS 2, crea un nodo "NumberPublisher" y lo ejecuta. 
 * Finalmente, finaliza ROS 2 y retorna 0 al sistema operativo.
 * 
 * @param argc Número de argumentos de la línea de comandos.
 * @param argv Argumentos de la línea de comandos.
 * @return 0 si la ejecución fue exitosa.
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<NumberPublisher>());  
    rclcpp::shutdown();  
    return 0;  
}
