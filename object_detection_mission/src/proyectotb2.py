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

# Definir waypoints de la ruta a seguir
waypoints = [
    ['1 Punto', (-2.0, 2.0)],
    ['2 Punto', (0.0, 5.0)],
    ['3 Punto', (2.0, 2.6)]
]

class RouteFollowingState(State):
    def __init__(self, waypoints):
        State.__init__(self, outcomes=['route_completed'])
        self.waypoints = waypoints
        self.detected_objects = []  # Lista para almacenar objetos detectados
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Esperando conexión con el servidor move_base...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Conexión establecida con el servidor move_base.")

        # Suscribirse al tópico de objetos detectados (ajustado para TurtleBot 2)
        rospy.Subscriber('/detected_objects', PoseStamped, self.object_callback)

    def object_callback(self, msg):
        """
        Callback para procesar objetos detectados.
        """
        # Extraer la posición global del objeto detectado
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.detected_objects.append({'x': x, 'y': y})
        rospy.loginfo(f"Objeto detectado en posición: x={x}, y={y}")

    def execute(self, userdata):
        rospy.loginfo("Iniciando recorrido por waypoints...")

        for i, (name, position) in enumerate(self.waypoints):
            # Calcular orientación hacia el siguiente waypoint
            if i < len(self.waypoints) - 1:
                next_position = self.waypoints[i + 1][1]
            else:  # Si es el último waypoint, apunta al primero para cerrar el ciclo
                next_position = self.waypoints[0][1]

            yaw = self.calcular_orientacion(position, next_position)
            orientation = quaternion_from_euler(0, 0, yaw)

            rospy.loginfo(f"Dirigiéndose a {name}: posición={position}, orientación={yaw} radianes")

            goal = self.crear_destino(position[0], position[1], orientation)
            self.move_base_client.send_goal(goal)

            # Esperar hasta alcanzar el destino o un tiempo máximo
            success = self.move_base_client.wait_for_result(rospy.Duration(60.0))  # Timeout de 60 segundos
            if not success:
                rospy.logwarn(f"No se pudo alcanzar el waypoint {name}. Reintentando...")
                continue
            rospy.loginfo(f"Waypoint {name} alcanzado.")

        # Mostrar objetos detectados al completar la ruta
        rospy.loginfo("Ruta completada. Objetos detectados:")
        for obj in self.detected_objects:
            rospy.loginfo(f"Objeto en posición: x={obj['x']}, y={obj['y']}")

        return 'route_completed'

    def calcular_orientacion(self, current_position, next_position):
        """
        Calcula el ángulo (yaw) necesario para que el robot mire hacia el siguiente waypoint.
        """
        dx = next_position[0] - current_position[0]
        dy = next_position[1] - current_position[1]
        return math.atan2(dy, dx)

    def crear_destino(self, x, y, orientation):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'  # Usar el marco 'map' en TurtleBot 2
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        return goal

def main():
    rospy.init_node('turtlebot2_route_follower')

    # Crear máquina de estados
    sm = StateMachine(outcomes=['mission_completed'])

    with sm:
        # Añadir estados a la máquina
        StateMachine.add('ROUTE_FOLLOWING', 
                         RouteFollowingState(waypoints),
                         transitions={'route_completed': 'mission_completed'})

    # Declaraciones para ver de una forma visual la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')  
    sis.start()

    # Ejecutar la máquina de estados
    sm.execute()
    rospy.spin()   

if __name__ == '__main__':
    main()