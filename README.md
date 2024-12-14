# ROS2_fundamentals

Este es un proyecto que contiene dos paquetes en Python y C++ con nodos que cubren los fundamentos del uso de ROS2.

- **prueba_robotics4_0**: Paquete de Python con nodos para ejecutar comunicación publicador/suscriptor, manejo de parámetros e integración de detección de objetos usando YOLOv8 con el repositorio para robots cuadrúpedos [CHAMP](https://github.com/chvmp/champ/tree/ros2).
- **prueba_robotics4_0cpp**: Paquete de C++ con nodos para ejecutar publicación e interfaz de servicios.

## Cómo instalar

Sigue los pasos a continuación para instalar y ejecutar los paquetes.

### Paso 1: Instalar dependencias

Primero, asegúrate de tener ROS 2 Humble instalado en tu sistema. Puedes seguir la guía oficial de instalación de ROS 2 para Ubuntu:

[Guía de instalación de ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Luego, instala las dependencias necesarias para los paquetes:

  1. **Instalar dependencias de ROS2 y paquetes de desarrollo:**

     ```bash
     sudo apt update
     sudo apt install ros-humble-desktop 
     sudo apt install python3-colcon-common-extensions python3-pip
     sudo apt install libyaml-cpp-dev  # Dependencia para C++
     sudo apt install python3-opencv
  2. **Instalar YOLOv8 (si no lo tienes instalado):**

Para la detección de objetos con YOLOv8, puedes seguir los pasos de instalación para su uso en Python:


    pip install ultralytics

### Paso 2: Compilar los paquetes

Clona el repositorio y navega a la carpeta del proyecto:
  
    git clone https://github.com/smilagros246/ROS2_fundamentals.git
    cd ROS2_fundamentals
  
Asegúrate de que todos los submódulos estén correctamente inicializados:

    git submodule update --init --recursive
Luego, construye los paquetes usando colcon:

    colcon build --symlink-install
Después de que la compilación se complete correctamente, fuente el entorno de trabajo:

    source install/setup.bash
### Paso 3: Correr los nodos
Ejecutar los nodos de Python:

Primero, asegúrate de que tu entorno esté correctamente configurado:

    source install/setup.bash
Luego, corre el nodo de Python (por ejemplo, el de prueba_robotics4_0):

    ros2 run prueba_robotics4_0 <nombre_del_nodo>
Los nombres disponibles son color_detection   object_detection    publicador    suscriptor    turtle_mover  
Ejecutar los nodos de C++:

De manera similar, puedes ejecutar los nodos en C++:

    ros2 run prueba_robotics4_0cpp <nombre_del_nodo_cpp>
Los nombres disponibles son number_publisher    service_client    service_server   
Asegúrate de reemplazar <nombre_del_nodo> o <nombre_del_nodo_cpp> por el nombre del nodo que deseas ejecutar, según corresponda.

### Requisitos
ROS Humble (Recomendado: versión oficial estable de ROS 2).
Python 3.10.12 o superior.

