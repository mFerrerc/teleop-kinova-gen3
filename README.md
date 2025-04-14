<div align="justify">

# Teleoperación de Kinova Gen3
Este proyecto presenta una interfaz de teleoperación diseñada para controlar un brazo robótico Kinova Kortex con pinza Robotiq 2f-140, todo ello en el simulador Gazebo. Se ha incluido la capacidad de realimentación háptica mediante el dispositivo PHANTOM Omni, de modo que el operador percibe las fuerzas y límites del entorno robótico en tiempo real. Con ello, se logra una interacción más realista y segura al manipular objetos y esquivar colisiones en el espacio de trabajo.

![image](https://github.com/user-attachments/assets/1d47630d-c30c-460a-9187-67eeb6d705ad)![image](https://github.com/user-attachments/assets/48cad7a4-f44a-40e7-b150-27b9c83d09d1)


La conexión maestro-esclavo se basa en un acoplamiento cinemático y un escalado que alinea el pequeño espacio del Omni con el volumen de movimiento del robot en Gazebo. Además, se integran dos modos de retroalimentación de fuerza: la Caja de Fuerza, que delimita el espacio operativo, y el Pozo de Gravedad, que favorece la estabilidad de la pinza ante movimientos de precisión. Para garantizar la detección de choques, se ha implementado un sistema de colisiones que registra contactos y eventos de agarre, ofreciendo una visión clara del estado del robot en todo momento.

La interfaz gráfica divide la información en paneles: el superior para controles, visualización de modos de operación y datos (como posiciones y velocidades), y el inferior dedicado a las cámaras virtuales (onboard, cenital, frontal), facilitando el monitoreo de la escena. Así, se consigue un entorno de teleoperación modular, adaptable a nuevas funciones y capaz de unificar la simulación, la retroalimentación háptica y la visualización de datos en un mismo flujo de trabajo.

Principales características:

- Control en posición entre el Omni Phantom (maestro) y el Kinova Kortex (esclavo).

- Escalado del espacio de trabajo y sincronización de orientaciones.

- Retroalimentación háptica con Caja de Fuerza y Pozo de Gravedad.

- Detección de colisiones y sincronización de agarre en Gazebo y RViz.

- Interfaz gráfica con paneles separados para información de teleoperación y visualización de cámaras.

### Vídeo Demostración

Se puede ver un vídeo de demostración de la gran mayoría el funcionamiento en este enlace:
https://www.youtube.com/watch?v=zVYfwNnUy7Q


## Lanzamiento de Aplicación

### Instalar Docker
Para instalar Docker en el ordenador con sistema operativo Ubuntu, se deben seguir
los pasos del link: https://docs.docker.com/engine/install/ubuntu/ , se recomienda
realizar los pasos de “Install using the repository”. Esta instalación permitirá el
uso de Docker en Ubuntu, la creación de imágenes Docker y el uso de
contenedores.
Nota: si se está trabajando en un ordenador con tarjeta gráfica dedicada y los drives
de la tarjeta gráfica están instalados correctamente, se recomienda instalar Nvidia
Docker.

- Install docker-nvidia sources
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && curl -s -L
https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - && curl -s -L
https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee
/etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
```
- Install nvidia-docker2
```
sudo apt-get install -y nvidia-docker2
sudo systemctl restart Docker
```
- Test docker with CUDA
```
sudo docker run --gpus all nvidia/cuda:11.5.2-base-ubuntu20.04 nvidia-smi
```
### Ejecutar contenedor

Una vez instalados los requisitos previos de Docker, es necesario compartir la
interfaz gráfica con el contenedor para permitir que las aplicaciones gráficas, como
las herramientas de ROS: RVIZ o Gazebo, se ejecuten correctamente. Para ello,
ejecuta el siguiente comando en la terminal:
```xhost +local:```

Este comando permite que las conexiones locales (es decir, conexiones desde el
mismo ordenador) se comunique con el servidor X, que es el sistema de ventanas utilizado en entornos gráficos de Ubuntu. Este paso es fundamental para poder
visualizar la interfaz gráfica desde el contenedor.

### Construcción del Contenedor Docker
Con las dependencias necesarias instaladas y la interfaz gráfica compartida, el
siguiente paso es construir el contenedor Docker que se puede obtener en el siguiente enlace:

https://drive.google.com/file/d/1G_lPhHabvC23i-ewcNKLahq5ox27L3Ht/view

Este archivo contiene las instrucciones necesarias para instalar automáticamente
todas las dependencias, repositorios y herramientas requeridas para el desarrollo de
esta práctica. Ejecuta el siguiente comando en la terminal:
```
docker load -i kinova-phantom_container_v2.tar
```

### Corriendo el contenedor
Tras identificar el puerto (para este ejemplo el resultado ha sido ttyACM0) se debe
lanzar el contenedor. Para ello se ejecuta el comando necesario para entrar al
contenedor lanzando el siguiente comando:
```
docker run -it --name=kinova-phantom_container \
--network host \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--device=/dev/ttyACM0:/dev/ttyACM0 \
--privileged \
-v /home/epvs/ros_kortex/:/home/kinova-phantom/catkin_ws/kinova/src/ros_kortex \
kinova-phantom_container_v2

```
Cada uno de los elementos puestos al lanzar el contenedor son esenciales para
poder desarrollar esta práctica. Se explica brevemente que es cada uno de estos
elementos:

### Abrir terminales en el contenedor
```
sudo docker exec -it kinova-phanthom_container /bin/bash
```

### Ejecutando los nodos del sensor Háptico

Para el sensor háptico por USB:
```
cd ~/catkin_ws/phanthom/ &&\
./start_omni_USB.sh
```

Para el sensor háptico por LAN:
```
cd ~/catkin_ws/phanthom/ &&\
./start_omni_LAN.sh
```

### Lanzar Aplicación
```
cd ~/catkin_ws/kinova/
source devel/setup.bash
roslaunch kortex_gazebo spawn_kortex_robot.launch start_rviz:=true
use_trajectory_controller:=false gripper:=robotiq_2f_140
```

</div>