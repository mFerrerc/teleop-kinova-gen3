# Teleoperación de Kinova Gen3
Este proyecto presenta una interfaz de teleoperación diseñada para controlar un brazo robótico Kinova Kortex con pinza Robotiq 2f-140, todo ello en el simulador Gazebo. Se ha incluido la capacidad de realimentación háptica mediante el dispositivo PHANTOM Omni, de modo que el operador percibe las fuerzas y límites del entorno robótico en tiempo real. Con ello, se logra una interacción más realista y segura al manipular objetos y esquivar colisiones en el espacio de trabajo.

La conexión maestro-esclavo se basa en un acoplamiento cinemático y un escalado que alinea el pequeño espacio del Omni con el volumen de movimiento del robot en Gazebo. Además, se integran dos modos de retroalimentación de fuerza: la Caja de Fuerza, que delimita el espacio operativo, y el Pozo de Gravedad, que favorece la estabilidad de la pinza ante movimientos de precisión. Para garantizar la detección de choques, se ha implementado un sistema de colisiones que registra contactos y eventos de agarre, ofreciendo una visión clara del estado del robot en todo momento.

La interfaz gráfica divide la información en paneles: el superior para controles, visualización de modos de operación y datos (como posiciones y velocidades), y el inferior dedicado a las cámaras virtuales (onboard, cenital, frontal), facilitando el monitoreo de la escena. Así, se consigue un entorno de teleoperación modular, adaptable a nuevas funciones y capaz de unificar la simulación, la retroalimentación háptica y la visualización de datos en un mismo flujo de trabajo.

Principales características:

- Control en posición entre el Omni Phantom (maestro) y el Kinova Kortex (esclavo).

- Escalado del espacio de trabajo y sincronización de orientaciones.

- Retroalimentación háptica con Caja de Fuerza y Pozo de Gravedad.

- Detección de colisiones y sincronización de agarre en Gazebo y RViz.

- Interfaz gráfica con paneles separados para información de teleoperación y visualización de cámaras.