# Examen Final Robótica 2024

Este repositorio contiene el código y los recursos necesarios para un proyecto que utiliza **ROS 2 Humble**, **OpenCV**, **cv_bridge**, el paquete **usb_cam**, y **Arduino** para programar una ESP32. El proyecto está orientado a aplicaciones robóticas avanzadas, como la integración de visión por computadora y control mediante microcontroladores.

---

## Requisitos del sistema

### Hardware:
- **Cámara USB** compatible con Ubunur 22.04.
- **ESP32** conectada y programable mediante Arduino.
- **PC o Laptop** con sistema operativo basado en Linux.

### Software:
- **ROS 2 Humble** instalado en el sistema. (Guía oficial: [Instalación de ROS 2](https://docs.ros.org/en/humble/Installation.html))
- **Arduino IDE** con soporte para ESP32. (Guía oficial: [Configuración de Arduino para ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- **OpenCV** instalado. (Guía: [Instalar OpenCV en Ubuntu](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html))
- Dependencias de Python y ROS 2 necesarias.

---

## Ejecucion

### 1. Clonar el repositorio
Clonar el repositorio en un workspace de ROS2 en una computadora y en una Raspberry
```bash
git clone https://github.com/Hinf1nity/examen_final_robotica_2024.git
colcon build
. install/setup.bashh
```

### 2. Arduino
Cargar el codigo de la carpeta arduino_codes a una esp32

### 4. ROS
Ejecutar en la computadora
```bash
ros2 run examen_final_robotica_2024 examen_3
```
Ejecutar en la Raspberry
```bash
ros2 launch examen_final_robotica_2024 examen_3.launch.py
```
