#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from smach import State, StateMachine
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

# Waypoints específicos para un entorno típico de TurtleBot 2
waypoints = [
    ['Inicio', (2.0, 3.0)],  # Punto de partida
    ['Punto A', (-2.0, 2.0)],  # Primera ubicación
    ['Punto B', (3.0, 6.0)],  # Segunda ubicación
    ['Punto C', (3.0, 1.0)],  # Tercera ubicación
]

# Estado para el control del robot
class WaitForOrder(State):
    def __init__(self):
        State.__init__(self, outcomes=['next_state', 'finish'])
        self.comand = None

    def execute(self, userdata):
        rospy.loginfo("Esperando un mensaje de voz...")
        self.comand = None  # Resetear el comando antes de ejecutar el estado

        # Subscripión al nodo de mensajes de voz
        sub_voz = rospy.Subscriber('/voice_commands', String, self.command_callback)

        # Esperar comando con un timeout razonable
        timeout = rospy.Time.now() + rospy.Duration(60)  # 60 segundos como límite
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("No se recibió ningún comando en el tiempo límite.")
                sub_voz.unregister()  # Cancelar la suscripción
                return 'finish'

            if self.comand == "trayectoria":
                rospy.loginfo("Comando 'trayectoria' recibido. Cambiando de estado.")
                sub_voz.unregister()
                return 'next_state'

            elif self.comand == "finalizar":
                rospy.loginfo("Comando 'finalizar' recibido. Terminando misión.")
                sub_voz.unregister()
                return 'finish'

            rate.sleep()

    def command_callback(self, msg):
        self.comand = msg.data.strip().lower()

# Estado para la navegación del robot
class TurtleBot2NavigationState(State):
    def __init__(self, waypoints, max_attempts=3):
        State.__init__(self, outcomes=['route_completed', 'navigation_failed'])
        self.waypoints = waypoints
        self.detected_objects = []  # Lista para almacenar objetos detectados
        self.detected_shapes = []  # Lista para almacenar formas detectadas
        self.max_attempts = max_attempts  # Número máximo de intentos por waypoint

        # Cliente de move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        rospy.loginfo("Conectando con el servidor move_base...")
        if not self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("No se pudo conectar con el servidor move_base.")
            return
        rospy.loginfo("Conexión establecida con move_base.")

        # Suscriptor para objetos detectados (posición y formas)
        rospy.Subscriber('/detected_objects', PoseStamped, self.object_callback)
        rospy.Subscriber('/detected_shapes', String, self.shape_callback)

    def object_callback(self, msg):
        """
        Callback para almacenar objetos detectados durante la navegación.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        rospy.loginfo(f"Objeto detectado en: x={x}, y={y}")
        self.detected_objects.append({'x': x, 'y': y})

    def shape_callback(self, msg):
        """
        Callback para almacenar las formas de objetos detectados.
        """
        shape = msg.data
        rospy.loginfo(f"Forma detectada: {shape}")
        self.detected_shapes.append(shape)

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación...")

        try:
            for i, (name, position) in enumerate(self.waypoints):
                rospy.loginfo(f"Iniciando navegación hacia {name} en {position}.")

                # Intentar alcanzar el waypoint con reintentos
                for attempt in range(1, self.max_attempts + 1):
                    rospy.loginfo(f"Intento {attempt}/{self.max_attempts} para alcanzar {name}.")
                    result = self.navegar_a_waypoint(name, position)

                    if result:
                        rospy.loginfo(f"Waypoint {name} alcanzado exitosamente.")
                        break
                    else:
                        rospy.logwarn(f"Fallo al alcanzar {name} en el intento {attempt}.")

                    if attempt == self.max_attempts:
                        rospy.logerr(f"Fallo definitivo al alcanzar {name} tras {self.max_attempts} intentos.")
                        return 'navigation_failed'

            # Resumen de objetos y formas detectados
            rospy.loginfo("Ruta completada. Resumen de detecciones:")
            for shape, obj in zip(self.detected_shapes, self.detected_objects):
                rospy.loginfo(f"Objeto: forma={shape}, posición: x={obj['x']}, y={obj['y']}")

            return 'route_completed'

        except Exception as e:
            rospy.logerr(f"Error durante la navegación: {e}")
            return 'navigation_failed'

    def navegar_a_waypoint(self, name, position):
        """
        Intenta navegar hacia un waypoint específico.
        """
        # Encontrar el índice del waypoint actual
        current_index = next((i for i, wp in enumerate(self.waypoints) if wp[0] == name and wp[1] == position), None)
        if current_index is None:
            rospy.logerr(f"Waypoint {name} con posición {position} no encontrado en la lista de waypoints.")
            return False

        # Calcular orientación hacia el siguiente waypoint
        next_index = (current_index + 1) % len(self.waypoints)
        next_position = self.waypoints[next_index][1]
        yaw = self.calcular_orientacion(position, next_position)
        orientation = quaternion_from_euler(0, 0, yaw)

        # Crear y enviar el objetivo
        goal = self.crear_destino(position[0], position[1], orientation)
        self.move_base_client.send_goal(goal)

        # Esperar resultado
        result = self.move_base_client.wait_for_result(rospy.Duration(120.0))  # Timeout de 120 segundos

        if not result:
            rospy.logwarn(f"Timeout al intentar alcanzar {name}.")
            return False

        estado = self.move_base_client.get_state()
        if estado == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            rospy.logwarn(f"Estado final inesperado al alcanzar {name}: {estado}")
            return False


    def calcular_orientacion(self, current_position, next_position):
        """
        Calcula ángulo de orientación entre waypoints.
        """
        dx = next_position[0] - current_position[0]
        dy = next_position[1] - current_position[1]
        return math.atan2(dy, dx)

    def crear_destino(self, x, y, orientation):
        """
        Crear goal de navegación para move_base.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Configuración de pose
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Orientación como cuaterniones
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        return goal


def main():
    rospy.init_node('turtlebot2_navigation_mission')

    # Aquí habrá que poner todas las suscripciones a los tópicos necesarios

    # Máquina de estados para navegación
    sm = StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:

        StateMachine.add('WAIT_ORDER',
                         WaitForOrder(),
                         transitions={
                             'next_state': 'ROUTE_FOLLOWING',
                             'finish': 'mission_completed'
                         })

        StateMachine.add('ROUTE_FOLLOWING',
                 TurtleBot2NavigationState(waypoints, max_attempts=3),
                 transitions={
                     'route_completed': 'WAIT_ORDER',
                     'navigation_failed': 'mission_failed'
                 })

    # Servidor de inspección de estados
    sis = smach_ros.IntrospectionServer('turtlebot2_navigation', sm, '/TB2_NAVIGATION')
    sis.start()

    # Ejecutar máquina de estados
    outcome = sm.execute()

    # Spinner para mantener el nodo activo
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo finalizado debido a una interrupción.")
    except Exception as e:
        rospy.logerr(f"Error inesperado: {e}")
