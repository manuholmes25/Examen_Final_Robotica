# Examen_Final_Robotica

Este repositorio contiene el workspace de ROS2 para la simulacion de un robot usando ArUcos.

## Requisitos Previos
* **Sistema Operativo:** Ubuntu 22.04 LTS.
* **Preparacion del Workspace e instalacion de dependecias:**
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions ros-humble-joint-state-publisher-gui ros-humble-xacro
  sudo apt install ros-humble-nav-msgs
  sudo apt install ros-${ROS_DISTRO}-rtabmap-ros
  
  
  #Descargar ZIP del proyecto y descomprimirlo para ejecutar el workspace
  
  
  #Crear directorio textures en el directorio palletizer_v1_description:
  mkdir -p /home/manuel/Pictures/Examen_Final_Robotica-main/src/palletizer_v1_description/textures

  #Compilar el proyecto
  cd ~/Examen_Final_Robotica-main
  colcon build
  source install/setup.bash #Se debe usar source install/setup.zsh si el bash fue modificado.

* **Comandos para ejecutar la simulacion:**
  ```bash
  #Primera terminal
  ros2 launch palletizer_v1_description gazebo.launch.py

  #Segunda terminal
  ros2 launch palletizer_process_node palletizer_process.launch.py
