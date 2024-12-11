#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from smach import State, StateMachine
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

# Waypoints específicos para un entorno típico de TurtleBot 2
waypoints = [
    ['Inicio', (0.0, 0.0)],  # Punto de partida
    ['Punto A', (2.0, 2.0)],  # Primera ubicación
    ['Punto B', (-2.0, 2.0)],  # Segunda ubicación
    ['Punto C', (-2.0, -2.0)],  # Tercera ubicación
]

class TurtleBot2NavigationState(State):
    def __init__(self, waypoints):
        State.__init__(self, outcomes=['route_completed', 'navigation_failed'])
        self.waypoints = waypoints
        self.detected_objects = []

        # Cliente de move_base específico para TurtleBot 2
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        rospy.loginfo("Conectando con el servidor move_base del TurtleBot 2...")
        if not self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("No se pudo conectar con el servidor move_base.")
            return
        rospy.loginfo("Conexión establecida con move_base.")

        # Suscriptor para objetos detectados (opcional)
        rospy.Subscriber('/turtlebot2/detected_objects', PoseStamped, self.object_callback)

    def object_callback(self, msg):
        """
        Callback para almacenar objetos detectados durante la navegación
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.detected_objects.append({'x': x, 'y': y})
        rospy.loginfo(f"Objeto detectado en: x={x}, y={y}")

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación del TurtleBot 2...")

        try:
            for i, (name, position) in enumerate(self.waypoints):
                # Calcular orientación hacia el siguiente waypoint
                next_index = (i + 1) % len(self.waypoints)
                next_position = self.waypoints[next_index][1]

                yaw = self.calcular_orientacion(position, next_position)
                orientation = quaternion_from_euler(0, 0, yaw)

                rospy.loginfo(f"Navegando a {name}: pos={position}, orientación={math.degrees(yaw)}°")

                goal = self.crear_destino(position[0], position[1], orientation)
                self.move_base_client.send_goal(goal)

                # Esperar resultado con timeout
                result = self.move_base_client.wait_for_result(rospy.Duration(120.0))

                if not result:
                    rospy.logerr(f"Fallo al alcanzar el waypoint {name}")
                    return 'navigation_failed'

                # Verificar estado del resultado
                estado = self.move_base_client.get_state()
                if estado != actionlib.GoalStatus.SUCCEEDED:
                    rospy.logwarn(f"Waypoint {name} no completado correctamente")
                    return 'navigation_failed'

                rospy.loginfo(f"Waypoint {name} alcanzado exitosamente.")

            # Resumen de objetos detectados
            rospy.loginfo("Ruta completada. Objetos detectados:")
            for obj in self.detected_objects:
                rospy.loginfo(f"Objeto en: x={obj['x']}, y={obj['y']}")

            return 'route_completed'

        except Exception as e:
            rospy.logerr(f"Error en navegación: {e}")
            return 'navigation_failed'

    def calcular_orientacion(self, current_position, next_position):
        """
        Calcula ángulo de orientación entre waypoints
        """
        dx = next_position[0] - current_position[0]
        dy = next_position[1] - current_position[1]
        return math.atan2(dy, dx)

    def crear_destino(self, x, y, orientation):
        """
        Crear goal de navegación para move_base
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Configuración de pose
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Orientación como cuaternión
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        return goal

def main():
    rospy.init_node('turtlebot2_navigation_mission')

    # Máquina de estados para navegación
    sm = StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:
        StateMachine.add('ROUTE_FOLLOWING',
                         TurtleBot2NavigationState(waypoints),
                         transitions={
                             'route_completed': 'mission_completed',
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
    main()
