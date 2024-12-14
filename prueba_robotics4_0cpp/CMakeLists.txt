cmake_minimum_required(VERSION 3.5)
project(prueba_robotics4_0cpp)

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar interfaces (servicio)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Square.srv"
  DEPENDENCIES std_msgs
)

# Crear ejecutables
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(service_server ${cpp_typesupport_target})

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs)
target_link_libraries(service_client ${cpp_typesupport_target})

# Instalar ejecutables
install(TARGETS
  number_publisher
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME})

# Asegurarse de que las interfaces sean instaladas
ament_export_dependencies(rosidl_default_runtime)
ament_package()
