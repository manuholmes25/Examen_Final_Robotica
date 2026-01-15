# Examen_Final_Robotica

Este repositorio contiene el workspace de ROS2 para el sistema de paletizado.

## Requisitos Previos
* **Sistema Operativo:** Ubuntu 22.04 LTS.
* **Preparacion del Workspace e instalacion de dependecias:**
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions ros-humble-joint-state-publisher-gui ros-humble-xacro

  #Crear el workspace y clonar
  mkdir -p ~/palletizer_ws/src
  cd ~/palletizer_ws/src
  git clone https://github.com/manuholmes25/Examen_Final_Robotica.git .

  #Instalar dependencias de ROS2
  cd ~/palletizer_ws
  rosdep install --from-paths src --ignore-src -r -y

  #Compilar el proyecto
  colcon build
  source install/setup.bash #Se debe usar source install/setup.zsh si el bash fue modificado.

* **Comandos para ejecutar la simulacion:**
  ```bash
  #Primera terminal
  ros2 launch palletizer_v1_description gazebo.launch.py

  #Segunda terminal
  ros2 launch palletizer_process_node palletizer_process.launch.py
