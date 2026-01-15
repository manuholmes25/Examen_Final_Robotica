# Examen_Final_Robotica

Este repositorio contiene el workspace de ROS2 para el sistema de paletizado.

## Requisitos Previos
* **Sistema Operativo:** Ubuntu 22.04 LTS.
* **Dependencias de Visión/Control:**
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions ros-humble-joint-state-publisher-gui ros-humble-xacro

  # 1. Crear el workspace y clonar
  mkdir -p ~/palletizer_ws/src
  cd ~/palletizer_ws/src
  git clone https://github.com/manuholmes25/Examen_Final_Robotica.git .

  # 2. Instalar dependencias de ROS
  cd ~/palletizer_ws
  rosdep install --from-paths src --ignore-src -r -y

  # 3. Compilar el proyecto
  colcon build --symlink-install
  source install/setup.bash
