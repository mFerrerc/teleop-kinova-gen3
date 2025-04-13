#!/usr/bin/env python3

# Importación de las bibliotecas necesarias
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np

# Constantes para el cálculo de la fuerza del pozo de gravedad
k = 1000.0  # Rigidez del resorte (N/m)
b = 0.001   # Coeficiente de amortiguación (Ns/m)
well_center = np.array([0.0, 0.0, 0.0])  # Posición del centro del pozo de gravedad
influence_radius = 1  # Radio de influencia del pozo (m)

# Publicador para aplicar la fuerza calculada al manipulador
force_pub = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=1)

# Límites de movimiento en los ejes X, Y y Z
LIM_X_MIN = -0.08
LIM_X_MAX = 0.08
LIM_Y_MIN = -0.06
LIM_Y_MAX = 0.018
LIM_Z_MIN = -0.073
LIM_Z_MAX = 0.048

def compute_gravity_well_force(omni_position):
    """
    Calcula la fuerza del pozo de gravedad en función de la posición actual del dispositivo háptico.
    La fuerza atrae al dispositivo hacia el centro del pozo de gravedad si está dentro del radio de influencia.

    Parámetros:
    - omni_position: numpy array con las coordenadas (x, y, z) de la posición actual del dispositivo

    Retorna:
    - force: numpy array con la fuerza (x, y, z) a aplicar en el dispositivo
    """
    displacement = well_center - omni_position  # Vector desplazamiento hacia el centro del pozo
    distance = np.linalg.norm(displacement)  # Distancia desde el omni al centro del pozo

    if distance < influence_radius:  # Dentro del radio de influencia
        force_magnitude = k * distance - b * np.linalg.norm(omni_position)  # Magnitud de la fuerza
        force_direction = displacement / distance  # Dirección de la fuerza normalizada
        force = force_magnitude * force_direction
    else:
        force = np.array([0.0, 0.0, 0.0])  # Fuera del radio de influencia, la fuerza es cero

    return force

def omni_position_callback(data):
    """
    Callback que se ejecuta al recibir un mensaje de posición del dispositivo háptico.
    Evalúa si la posición está fuera de los límites permitidos y publica fuerzas correctivas.

    Parámetros:
    - data: mensaje PoseStamped con la posición actual del dispositivo háptico
    """
    # Convierte la posición del omni en un array numpy
    omni_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    # Fuerza máxima aplicada si se exceden los límites
    f = 1000

    # Verifica los límites en cada eje y publica fuerzas correctivas
    if data.pose.position.x < LIM_X_MIN:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = f
        force_msg.wrench.force.y = 0
        force_msg.wrench.force.z = 0
        force_pub.publish(force_msg)
    elif data.pose.position.x > LIM_X_MAX:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = -f
        force_msg.wrench.force.y = 0
        force_msg.wrench.force.z = 0
        force_pub.publish(force_msg)
    elif data.pose.position.y < LIM_Y_MIN:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = 0
        force_msg.wrench.force.y = f
        force_msg.wrench.force.z = 0
        force_pub.publish(force_msg)
    elif data.pose.position.y > LIM_Y_MAX:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = 0
        force_msg.wrench.force.y = -f
        force_msg.wrench.force.z = 0
        force_pub.publish(force_msg)
    elif data.pose.position.z < LIM_Z_MIN:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = 0
        force_msg.wrench.force.y = 0
        force_msg.wrench.force.z = f
        force_pub.publish(force_msg)
    elif data.pose.position.z > LIM_Z_MAX:
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = 0
        force_msg.wrench.force.y = 0
        force_msg.wrench.force.z = -f
        force_pub.publish(force_msg)
    else:
        # Si está dentro de los límites, no aplica fuerza
        force_msg = WrenchStamped()
        force_msg.wrench.force.x = 0
        force_msg.wrench.force.y = 0
        force_msg.wrench.force.z = 0
        force_pub.publish(force_msg)

def main():
    """
    Función principal que inicializa el nodo de ROS y suscribe el nodo al tópico de posición del dispositivo háptico.
    """
    # Inicializa el nodo de ROS
    rospy.init_node('gravity_well', anonymous=True)
    
    # Suscripción al tópico de posición del omni
    rospy.Subscriber('/arm/measured_cp', PoseStamped, omni_position_callback)

    # Mantiene el nodo en funcionamiento
    rospy.spin()

if __name__ == '__main__':
    main()
