#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Interfaz de Teleoperación con PyQt5 que se integra con ROS para mostrar:
 - Modo de control (posicional o de velocidad).
 - Información de la pinza (abierta o cerrada) a través de LEDs.
 - Coordenadas y velocidades de los robots maestro y esclavo.
 - Información de contacto y peso del objeto manipulado.
 - Tres vistas de cámara (Onboard, Cenital, Left).

La aplicación se ejecuta como un nodo de ROS y, al mismo tiempo, 
como una GUI de PyQt5, empleando múltiples suscriptores para recibir 
información y publicadores para enviar comandos (ej. modo de teleoperación).
"""

import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QGroupBox, QFrame
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap

import rospy
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import WrenchStamped, PoseStamped

class TeleoperationInterface(QMainWindow):
    """
    Clase que construye una GUI con PyQt5, representando una interfaz 
    de teleoperación que consume y publica tópicos de ROS:
      - Muestra y permite seleccionar el modo de control (posicional o velocidad).
      - Indica el estado de la pinza (abierta o cerrada).
      - Muestra coordenadas y velocidades de maestro y esclavo.
      - Muestra información de contacto y peso de un objeto manipulado.
      - Provee vistas de cámara en tiempo real (onboard, cenital y left).
    """
    def __init__(self):
        super().__init__()

        # Título y ajuste de ventana
        self.setWindowTitle("Interfaz de Teleoperación")
        self.showMaximized()  # Inicia la aplicación en modo pantalla completa

        # Inicializar el nodo ROS
        rospy.init_node('teleoperation_interface', anonymous=True)

        # Inicializar CvBridge para convertir mensajes de imagen de ROS a OpenCV
        self.bridge = CvBridge()

        # Crear un widget central y layout principal
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Crear panel superior (información de teleoperación, maestro/esclavo, contacto/pinza)
        top_panel = self.create_top_panel()
        main_layout.addWidget(top_panel)

        # Crear panel inferior (vistas de cámara)
        camera_panel = self.create_camera_panel()
        main_layout.addWidget(camera_panel)

        # Configurar suscripciones a tópicos de ROS
        self.init_ros_subscriptions()

    # ----------------------------------------------------
    # Métodos de construcción de la interfaz (UI)
    # ----------------------------------------------------
    def create_top_panel(self):
        """
        Construye el panel superior que incluye:
         - Panel izquierdo: Modo de teleoperación y estado de la pinza.
         - Panel central/derecho: Información de coordenadas (maestro/esclavo).
         - Panel de contacto (peso, contactos, etc.).
        Retorna un contenedor QWidget con un QHBoxLayout.
        """
        top_layout = QHBoxLayout()

        # Columna izquierda: modos de teleoperación y LED de pinza
        self.teleop_info_panel = self.create_teleop_panel()
        top_layout.addWidget(self.teleop_info_panel)

        # Panel central: información maestro/esclavo
        self.teleop_info_panel = self.create_info_panel()
        top_layout.addWidget(self.teleop_info_panel)
        
        # Columna derecha: info de contacto, peso y estado de la pinza
        self.contact_info_panel = self.create_contact_info_panel()
        top_layout.addWidget(self.contact_info_panel)

        container = QWidget()
        container.setLayout(top_layout)
        return container

    def create_teleop_panel(self):
        """
        Crea un panel vertical que agrupa:
         - Grupo de botones para seleccionar el modo de control (posicional o velocidad).
         - Grupo con LEDs que indican estado de la pinza (abierta o cerrada).
        """
        teleop_layout = QVBoxLayout()
        
        # Primera fila: botones de modo de control
        self.teleop_info_panel = self.fila1()
        teleop_layout.addWidget(self.teleop_info_panel)

        # Segunda fila: LEDs de la pinza
        self.teleop_info_panel = self.fila2()
        teleop_layout.addWidget(self.teleop_info_panel)        
        
        container = QWidget()
        container.setLayout(teleop_layout)
        return container

    def fila1(self):
        """
        Crea un QGroupBox que contiene los botones para seleccionar modo de control 
        (Posición o Velocidad). Al pulsar un botón, se publica a /modo_teleoperacion 
        con la cadena correspondiente.
        """
        group_box = QGroupBox("Modo de Control")
        layout = QVBoxLayout()

        # Disposición horizontal para los botones
        mode_layout = QHBoxLayout()
        self.position_mode_btn = QPushButton("Posición")
        self.velocity_mode_btn = QPushButton("Velocidad")

        # Conectar pulsaciones a una función que publica el modo al tópico "/modo_teleoperacion"
        self.position_mode_btn.clicked.connect(lambda: self.publish_mode("posicion"))
        self.velocity_mode_btn.clicked.connect(lambda: self.publish_mode("velocidad"))

        mode_layout.addWidget(self.position_mode_btn)
        mode_layout.addWidget(self.velocity_mode_btn)
        layout.addLayout(mode_layout)

        group_box.setLayout(layout)
        return group_box
    
    def fila2(self):
        """
        Crea un QGroupBox que muestra dos LEDs (labels) para la pinza:
         - 'Abierta' (verde)
         - 'Cerrada' (rojo)
        Se cambian dinámicamente a través de las suscripciones ROS.
        """
        group_box = QGroupBox("Estado de la pinza")
        layout = QVBoxLayout()
        
        # Layout horizontal para los dos LED
        led_layout = QHBoxLayout()

        # LED verde: indica "Abierta"
        self.led_open = QLabel("Abierta")
        self.led_open.setFixedSize(100, 40)
        self.led_open.setAlignment(Qt.AlignCenter)
        self.led_open.setStyleSheet("background-color: darkgreen; color: white; border-radius: 10px; font-weight: bold;")
        led_layout.addWidget(self.led_open)

        # LED rojo: indica "Cerrada"
        self.led_closed = QLabel("Cerrada")
        self.led_closed.setFixedSize(100, 40)
        self.led_closed.setAlignment(Qt.AlignCenter)
        self.led_closed.setStyleSheet("background-color: darkred; color: white; border-radius: 10px; font-weight: bold;")
        led_layout.addWidget(self.led_closed)

        layout.addLayout(led_layout)
        group_box.setLayout(layout)
        return group_box

    def create_info_panel(self):
        """
        Crea un QGroupBox que muestra información de teleoperación:
         - Coordenadas y velocidad del maestro
         - Coordenadas y velocidad del esclavo
        """
        group_box = QGroupBox("Información de Teleoperación")
        layout = QVBoxLayout()

        # Coordenadas y velocidades del maestro
        self.master_coords_label = QLabel("Coordenadas Maestro: \nx: 0.0 \ny: 0.0 \nz: 0.0")
        self.master_vel_label = QLabel("Velocidad Maestro: \nx: 0.0 \ny: 0.0 \nz: 0.0")
        layout.addWidget(self.master_coords_label)
        layout.addWidget(self.master_vel_label)

        # Coordenadas y velocidades del esclavo
        self.slave_coords_label = QLabel("Coordenadas Esclavo: \nx: 0.0 \ny: 0.0 \nz: 0.0")
        self.slave_vel_label = QLabel("Velocidad Esclavo: \nx: 0.0 \ny: 0.0 \nz: 0.0")
        layout.addWidget(self.slave_coords_label)
        layout.addWidget(self.slave_vel_label)

        group_box.setLayout(layout)
        return group_box

    def create_contact_info_panel(self):
        """
        Crea un QGroupBox con información de contacto y la pinza:
         - Peso del objeto
         - Número de contactos
         - Coordenadas del contacto
        """
        group_box = QGroupBox("Información de Contacto y Pinza")
        layout = QVBoxLayout()

        # Etiqueta de peso
        self.object_weight_label = QLabel("Peso del Objeto: 0.0 kg")
        layout.addWidget(self.object_weight_label)

        # Etiqueta del número de contactos
        self.num_contacts_label = QLabel("Número de Contactos: 0")
        layout.addWidget(self.num_contacts_label)

        # Etiqueta para las coordenadas del contacto
        self.contact_coords_label = QLabel("Coordenadas Contacto: \nx: 0.0 \ny: 0.0 \nz: 0.0")
        layout.addWidget(self.contact_coords_label)

        group_box.setLayout(layout)
        return group_box

    def create_camera_panel(self):
        """
        Crea un QGroupBox que contiene tres vistas de cámara (Onboard, Cenital, Left),
        cada una con su QLabel para mostrar la imagen en tiempo real.
        """
        group_box = QGroupBox("Cámaras")
        layout = QHBoxLayout()

        # Para cada cámara se crea un widget con su layout
        self.onboard_camera_widget = self.create_camera_view("Onboard")
        self.cenital_camera_widget = self.create_camera_view("Cenital")
        self.left_camera_widget = self.create_camera_view("Left")

        # Se añaden al layout horizontal principal
        layout.addWidget(self.onboard_camera_widget)
        layout.addWidget(self.cenital_camera_widget)
        layout.addWidget(self.left_camera_widget)

        group_box.setLayout(layout)
        return group_box

    def create_camera_view(self, camera_name):
        """
        Crea un QWidget con un QLabel que funciona como lienzo para la imagen, 
        además de una etiqueta con el nombre de la cámara.
        """
        camera_layout = QVBoxLayout()

        # Etiqueta con el nombre de la cámara
        camera_label = QLabel(camera_name)
        camera_label.setAlignment(Qt.AlignCenter)
        camera_label.setStyleSheet("font-weight: bold;")
        camera_layout.addWidget(camera_label)

        # Vista de la cámara
        camera_view = QLabel()
        camera_view.setFixedSize(640, 480)  # Tamaño fijo para la imagen
        camera_view.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(camera_view)

        # Contenedor final
        camera_widget = QWidget()
        camera_widget.setLayout(camera_layout)
        return camera_widget

    # ----------------------------------------------------
    # Suscripciones a ROS: Tópicos y callbacks
    # ----------------------------------------------------
    def init_ros_subscriptions(self):
        """
        Inicializa todos los suscriptores de ROS necesarios:
          - Contacto (/num_contactos, /media_contactos)
          - Peso del objeto (/peso_objeto)
          - Estado de la pinza (/estado_pinza)
          - Coordenadas y velocidades de maestro y esclavo
          - Tópicos de imagen para las 3 cámaras
        """
        rospy.Subscriber("/num_contactos", Int32, self.update_num_contacts)
        rospy.Subscriber("/media_contactos", Point, self.update_contact_coords)
        rospy.Subscriber("/peso_objeto", Float32, self.update_object_weight)
        rospy.Subscriber("/estado_pinza", String, self.update_gripper_state)

        rospy.Subscriber("/posicion_maestro", Vector3, self.update_master_coords)
        rospy.Subscriber("/velocidad_maestro", Vector3, self.update_master_vel)
        rospy.Subscriber("/posicion_esclavo", Vector3, self.update_slave_coords)
        rospy.Subscriber("/my_gen3/in/cartesian_velocity", Vector3, self.update_slave_vel)

        # Suscripciones a los tópicos de imagen
        rospy.Subscriber('/my_gen3/robot_camera/onboard', Image, self.callback_camera_onboard)
        rospy.Subscriber('/robot_camera/Cenital', Image, self.callback_camera_cenital)
        rospy.Subscriber('/robot_camera/Left', Image, self.callback_camera_left)

    def callback_camera_onboard(self, msg):
        """
        Callback para la cámara Onboard.
        Se recibe una imagen de ROS y se actualiza la vista asociada.
        En este caso, se rota 180° si es necesario.
        """
        # En la prueba, no se aplica rotación. Se deja rotate=False.
        self.update_camera_image(self.onboard_camera_widget.findChild(QLabel), msg, rotate=False)

    def callback_camera_cenital(self, msg):
        """
        Callback para la cámara Cenital.
        """
        self.update_camera_image(self.cenital_camera_widget.findChild(QLabel), msg)

    def callback_camera_left(self, msg):
        """
        Callback para la cámara Left.
        """
        self.update_camera_image(self.left_camera_widget.findChild(QLabel), msg)

    def update_camera_image(self, camera_label, msg, rotate=False):
        """
        Convierte la imagen recibida en formato ROS a un QPixmap y la muestra en un QLabel.
        Si 'rotate=True', rota la imagen 180° usando OpenCV.
        """
        # Convertir de ROS a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Rotar la imagen si se requiere
        if rotate:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        # Convertir de OpenCV a QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

        # Ajustar la imagen (escalada) al QLabel
        pixmap = QPixmap.fromImage(qt_image).scaled(550, 400, Qt.KeepAspectRatio)
        camera_label.setPixmap(pixmap)
    
    # ----------------------------------------------------
    # Publicaciones a ROS
    # ----------------------------------------------------
    def publish_mode(self, mode):
        """
        Publica en el tópico /modo_teleoperacion para indicar 
        si se está controlando en "posicion" o "velocidad".
        """
        pub = rospy.Publisher("/modo_teleoperacion", String, queue_size=10)
        pub.publish(mode)

    # ----------------------------------------------------
    # Callbacks para actualizar la interfaz con datos de ROS
    # ----------------------------------------------------
    def update_num_contacts(self, msg):
        """
        Actualiza la etiqueta con el número de contactos detectados.
        """
        self.num_contacts_label.setText(f"Número de Contactos: {msg.data}")

    def update_contact_coords(self, msg):
        """
        Actualiza la etiqueta con las coordenadas del contacto.
        """
        self.contact_coords_label.setText(f"Coordenadas Contacto: \nx: {msg.x:.2f} \ny: {msg.y:.2f} \nz: {msg.z:.2f}")

    def update_object_weight(self, msg):
        """
        Actualiza la etiqueta con el peso del objeto manipulado.
        """
        self.object_weight_label.setText(f"Peso del Objeto: {msg.data:.2f} kg")

    def update_gripper_state(self, msg):
        """
        Actualiza los LEDs de estado de la pinza según 
        el mensaje recibido (/estado_pinza).
        """
        if msg.data.lower() == "abierta":
            # LED verde brillante => Abierta
            self.led_open.setStyleSheet("background-color: lime; color: black; border-radius: 10px; font-weight: bold;")
            # LED rojo oscuro => Cerrada
            self.led_closed.setStyleSheet("background-color: darkred; color: white; border-radius: 10px; font-weight: bold;")
        elif msg.data.lower() == "cerrada":
            # LED verde oscuro => Abierta
            self.led_open.setStyleSheet("background-color: darkgreen; color: white; border-radius: 10px; font-weight: bold;")
            # LED rojo brillante => Cerrada
            self.led_closed.setStyleSheet("background-color: red; color: black; border-radius: 10px; font-weight: bold;")
        else:
            # Estado desconocido, ambos en color oscuro
            self.led_open.setStyleSheet("background-color: darkgreen; color: white; border-radius: 10px; font-weight: bold;")
            self.led_closed.setStyleSheet("background-color: darkred; color: white; border-radius: 10px; font-weight: bold;")

    def update_master_coords(self, msg):
        """
        Muestra las coordenadas del maestro con precisión de 5 decimales.
        """
        self.master_coords_label.setText(
            f"Coordenadas Maestro: \nx: {msg.x:.5f} \ny: {msg.y:.5f} \nz: {msg.z:.5f}"
        )

    def update_master_vel(self, msg):
        """
        Muestra la velocidad del maestro con precisión de 2 decimales.
        """
        self.master_vel_label.setText(
            f"Velocidad Maestro: \nx: {msg.x:.2f} \ny: {msg.y:.2f} \nz: {msg.z:.2f}"
        )

    def update_slave_coords(self, msg):
        """
        Muestra las coordenadas del esclavo con precisión de 5 decimales.
        """
        self.slave_coords_label.setText(
            f"Coordenadas Esclavo: \nx: {msg.x:.5f} \ny: {msg.y:.5f} \nz: {msg.z:.5f}"
        )

    def update_slave_vel(self, msg):
        """
        Muestra la velocidad del esclavo con precisión de 2 decimales.
        """
        self.slave_vel_label.setText(
            f"Velocidad Esclavo: \nx: {msg.x:.2f} \ny: {msg.y:.2f} \nz: {msg.z:.2f}"
        )


if __name__ == "__main__":
    # Crear la instancia de la aplicación PyQt
    app = QApplication(sys.argv)
    # Instanciar la interfaz de teleoperación
    window = TeleoperationInterface()
    window.show()
    # Ejecutar el bucle de eventos de PyQt
    sys.exit(app.exec_())
