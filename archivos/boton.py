#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState, Joy
from visualization_msgs.msg import Marker
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import numpy as np
from std_msgs.msg import String

# Coordenadas objetivo y posiciones iniciales
goal_x = 0.39
goal_y = 0.60
goal_z = 0.30
rx = 0.39
ry = 0.6
rz = 0.3

# Constantes del pozo de gravedad
k = 100.0  # Rigidez del resorte (N/m)
b = 0.01   # Coeficiente de amortiguación (Ns/m)

# Radio de influencia del pozo de gravedad
influence_radius = 0.05 # Radio de influencia del pozo (m)

# Publicador para aplicar la fuerza
force_pub = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=1)

# Límites de movimiento
LIM_X_MIN = -0.08
LIM_X_MAX = 0.08
LIM_Y_MIN = -0.06
LIM_Y_MAX = 0.018
LIM_Z_MIN = -0.073
LIM_Z_MAX = 0.048

def compute_gravity_well_force(omni_position, well_center):
    """
    Calcula la fuerza del pozo de gravedad en función de la posición actual del
    dispositivo háptico (`omni_position`). La fuerza atrae al dispositivo hacia el
    centro del pozo de gravedad siempre que esté dentro del radio de influencia.

    Parámetros:
    - omni_position: numpy array con las coordenadas (x, y, z) de la posición actual del dispositivo

    Retorna:
    - force: numpy array con la fuerza (x, y, z) a aplicar en el dispositivo
    """
    displacement = well_center - omni_position   # Desplazamiento hacia el centro del pozo
    distance = np.linalg.norm(displacement)      # Distancia al centro del pozo

    if distance < influence_radius: # Si está dentro del radio de influencia
        # Calcula la magnitud de la fuerza proporcional a la distancia
        force_magnitude = k * distance - b * np.linalg.norm(omni_position)
        force_direction = displacement / distance  # Dirección normalizada hacia el pozo
        force = force_magnitude * force_direction
    else:
        # Fuera del radio de influencia, la fuerza es cero
        force = np.array([0.0, 0.0, 0.0])

    return force

class ExampleMoveItTrajectories(object):
    """Clase para controlar trayectorias usando MoveIt!"""
    def __init__(self):
        # Inicialización de MoveIt y el nodo de ROS
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            # Comprueba si hay un actuador tipo gripper conectado
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""

            # Grados de libertad del robot
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Interfaces de MoveIt
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        """Alcanza una posición predefinida usando MoveIt."""
        arm_group = self.arm_group
        arm_group.set_named_target(target)  # Fija el objetivo
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()  # Planifica la trayectoria
        return arm_group.execute(trajectory_message, wait=True)  # Ejecuta la trayectoria

    def reach_joint_angles(self, tolerance):
        """Mueve las articulaciones del robot a ángulos específicos."""
        arm_group = self.arm_group
        success = True

        # Obtiene las posiciones actuales de las articulaciones
        joint_positions = arm_group.get_current_joint_values()

        # Configura la tolerancia de los ángulos
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Fija los ángulos objetivo en función de los grados de libertad
        if self.degrees_of_freedom == 7:
            joint_positions[0] = pi/2
            joint_positions[1] = 0
            joint_positions[2] = pi/4
            joint_positions[3] = -pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            joint_positions[6] = 0.2
        elif self.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi/2
            joint_positions[3] = pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
        arm_group.set_joint_value_target(joint_positions)  # Aplica los valores objetivo

        # Planifica y ejecuta la trayectoria
        success &= arm_group.go(wait=True)

        return success

    def get_cartesian_pose(self):
        """Obtiene la posición cartesiana actual del robot."""
        arm_group = self.arm_group
        pose = arm_group.get_current_pose()
        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        """Mueve el robot a una posición cartesiana específica."""
        arm_group = self.arm_group
        arm_group.set_goal_position_tolerance(tolerance)  # Fija la tolerancia

        if constraints is not None:
            arm_group.set_path_constraints(constraints)  # Aplica restricciones

        arm_group.set_pose_target(pose)  # Fija la posición objetivo
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        """Mueve el gripper a una posición relativa específica."""
        gripper_group = self.gripper_group

        # Obtiene los límites del gripper
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            # Calcula y mueve el gripper a la posición relativa
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False

# Variables globales para los botones
pulsado1 = False
pulsado2 = False

def callbackbt1(data):
    """Callback para el botón 1."""
    global pulsado1
    if data.buttons[0] == 1:
        pulsado1 = True
    else:
        pulsado1 = False

def callbackbt2(data):
    """Callback para el botón 2."""
    global pulsado2
    if data.buttons[0] == 1:
        pulsado2 = True
    else:
        pulsado2 = False

def main():
    """Función principal."""
    global goal_x, goal_y, goal_z, pulsado1, pulsado2
    example = ExampleMoveItTrajectories()  # Instancia la clase de control

    success = example.is_init_success  # Verifica la inicialización
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    rospy.Subscriber('/arm/button1', Joy, callbackbt1)  # Suscriptor para el botón 1
    rospy.Subscriber('/arm/button2', Joy, callbackbt2)  # Suscriptor para el botón 2
    pub = rospy.Publisher('/estado_pinza', String, queue_size=10)  # Publicador del estado de la pinza

    while not rospy.is_shutdown():
        try:
            if example.is_gripper_present and success and pulsado1:  # Si el botón 1 está presionado
                rospy.loginfo("Cerrando la pinza...")
                pub.publish("cerrada")
                success &= example.reach_gripper_position(1)  # Cierra la pinza
                pulsado1 = False

            if example.is_gripper_present and success and pulsado2:  # Si el botón 2 está presionado
                rospy.loginfo("Abriendo la pinza...")
                pub.publish("abierta")
                success &= example.reach_gripper_position(0)  # Abre la pinza
                pulsado2 = False

        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()

