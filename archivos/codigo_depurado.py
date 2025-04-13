#!/usr/bin/env python3

# Importación de bibliotecas necesarias para ROS y cálculos matemáticos
import sys
import time
import rospy
import moveit_commander
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import numpy as np
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler

# Coordenadas objetivo iniciales para el manipulador
goal_x = 0.2  # Coordenada x objetivo
goal_y = 0.3  # Coordenada y objetivo
goal_z = 0.4  # Coordenada z objetivo
rx = 0.39
ry = 0.6
rz = 0.3

# Constantes para el cálculo de fuerzas en un pozo de gravedad
k = 100.0  # Rigidez del resorte (N/m)
b = 0.01   # Coeficiente de amortiguación (Ns/m)

influence_radius = 0.05  # Radio de influencia del pozo de gravedad (m)

# Publicador para aplicar fuerzas en el manipulador
force_pub = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=1)

# Límites de movimiento en los ejes X, Y, Z
LIM_X_MIN = -0.08
LIM_X_MAX = 0.08
LIM_Y_MIN = -0.06
LIM_Y_MAX = 0.018
LIM_Z_MIN = -0.073
LIM_Z_MAX = 0.048

# Fuerza aplicada cuando se encuentra en el radio de influencia
f = 10

def compute_gravity_well_force(omni_position, well_center):
    """
    Calcula la fuerza que atrae al dispositivo hacia el centro de un pozo de gravedad.
    Parámetros:
    - omni_position: numpy array con la posición actual (x, y, z).
    - well_center: numpy array con la posición del centro del pozo.
    Retorna:
    - force: numpy array con las componentes de la fuerza (x, y, z).
    """
    displacement = well_center - omni_position  # Vector desplazamiento hacia el centro
    print("well_center ", well_center)
    print("omni_position ", omni_position)
    distance = np.linalg.norm(displacement)  # Magnitud del desplazamiento

    if distance < influence_radius:  # Dentro del radio de influencia
        force_magnitude = k * distance - b * np.linalg.norm(omni_position)  # Magnitud de la fuerza
        force_direction = displacement / distance  # Dirección normalizada
        # Se podría activar una fuerza en base a condiciones específicas
        force = np.array([0.0, 0.0, 0.0])
    else:
        force = np.array([0.0, 0.0, 0.0])  # Fuera del radio de influencia

    return force

class ExampleMoveItTrajectories(object):
    """Clase para manejar trayectorias de un manipulador usando MoveIt!"""
    def __init__(self):
        # Inicializa MoveIt y el nodo de ROS
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            # Verifica si hay un gripper conectado
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""

            # Define los grados de libertad del robot
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Configuración de MoveIt
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20
            )

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        """Alcanza una posición predefinida."""
        arm_group = self.arm_group
        arm_group.set_named_target(target)  # Fija el objetivo
        success_flag, trajectory_message, _, _ = arm_group.plan()  # Planifica
        return arm_group.execute(trajectory_message, wait=True)  # Ejecuta

    def get_cartesian_pose(self):
        """Obtiene la posición cartesiana actual."""
        return self.arm_group.get_current_pose().pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        """Alcanza una posición cartesiana específica con tolerancia opcional."""
        self.arm_group.set_goal_position_tolerance(tolerance)
        if constraints is not None:
            self.arm_group.set_path_constraints(constraints)
        self.arm_group.set_pose_target(pose)
        return self.arm_group.go(wait=True)

def callback(data):
    """Callback para procesar la posición del maestro y calcular la fuerza del pozo de gravedad."""
    global goal_x, goal_y, goal_z, rx, ry, rz, pubpos, pubpos2

    # Obtiene la posición del maestro
    omni_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    x = data.pose.position.x
    y = data.pose.position.z
    z = data.pose.position.y

    # Publica la posición del maestro
    posicion_maestro = Vector3(x, y, z)
    pubpos.publish(posicion_maestro)

    # Calcula la posición transformada
    transPos(x, y, z)
    p_x, p_y, p_z = transPosInversa(rx, ry, rz)
    print("Posición transformada inversa x:", p_x, " y:", p_y, " z:", p_z)

    # Calcula la fuerza del pozo de gravedad
    well_center = np.array([p_x, p_z, p_y])
    force = compute_gravity_well_force(omni_position, well_center)

    # Publica la fuerza calculada
    force_msg = WrenchStamped()
    force_msg.wrench.force.x = force[0]
    force_msg.wrench.force.y = force[1]
    force_msg.wrench.force.z = force[2]
    force_pub.publish(force_msg)

def transPos(x, y, z):
    """Transforma la posición del maestro al rango del esclavo."""
    global goal_x, goal_y, goal_z
    goal_x = 0.2 + ((x - LIM_X_MIN) / (LIM_X_MAX - LIM_X_MIN)) * (0.6 - 0.2)
    goal_y = 0.15 + ((y - LIM_Y_MIN) / (LIM_Y_MAX - LIM_Y_MIN)) * (0.6 - 0.15)
    goal_z = 0.3 + ((z - LIM_Z_MIN) / (LIM_Z_MAX - LIM_Z_MIN)) * (0.65 - 0.3)

def transPosInversa(x, y, z):
    """Transforma posiciones del esclavo al maestro."""
    p_x = LIM_X_MIN + ((x - 0.2) / (0.6 - 0.2)) * (LIM_X_MAX - LIM_X_MIN)
    p_y = LIM_Y_MIN + ((y - 0.15) / (0.6 - 0.15)) * (LIM_Y_MAX - LIM_Y_MIN)
    p_z = LIM_Z_MIN + ((z - 0.3) / (0.65 - 0.3)) * (LIM_Z_MAX - LIM_Z_MIN)
    return p_x, p_y, p_z

def main():
    """Función principal para iniciar el control del robot."""
    global goal_x, goal_y, goal_z, pubpos, pubpos2
    example = ExampleMoveItTrajectories()
    success = example.is_init_success

    rospy.Subscriber('/arm/measured_cp', PoseStamped, callback)  # Suscriptor de posiciones
    pubpos = rospy.Publisher('/posicion_maestro', Vector3, queue_size=10)
    pubpos2 = rospy.Publisher('/posicion_esclavo', Vector3, queue_size=10)

    while not rospy.is_shutdown():
        try:
            actual_pose = example.get_cartesian_pose()  # Posición actual del robot
            rx, ry, rz = actual_pose.position.x, actual_pose.position.y, actual_pose.position.z

            # Actualiza la posición objetivo
            actual_pose.position.x = goal_x
            actual_pose.position.y = goal_y
            actual_pose.position.z = goal_z

            # Publica la posición del esclavo
            posicion_esclavo = Vector3(goal_x, goal_y, goal_z)
            pubpos2.publish(posicion_esclavo)

            # Establece la orientación deseada
            roll, pitch, yaw = 1.57, 3.14, 3.14  # Orientación deseada
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            actual_pose.orientation.x = quaternion[0]
            actual_pose.orientation.y = quaternion[1]
            actual_pose.orientation.z = quaternion[2]
            actual_pose.orientation.w = quaternion[3]

            success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()

