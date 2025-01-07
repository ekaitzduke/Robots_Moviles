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

detected_objects = []  # Lista global para almacenar objetos detectados

# Función global para crear el movimiento al destino indicado
def crear_destino(x, y):
    """
    Crea un objetivo de navegación para move_base.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Orientación neutral
    return goal

# Estado para esperar comandos de voz
class WaitForOrder(State):
    def __init__(self):
        State.__init__(self, outcomes=['next_state', 'go_to_objects', 'finish'])
        self.comand = None

    def execute(self, userdata):
        rospy.loginfo("Esperando un mensaje de voz...")
        self.comand = None  # Resetear el comando antes de ejecutar el estado
        sub_voz = rospy.Subscriber('/voice_commands', String, self.command_callback)

        timeout = rospy.Time.now() + rospy.Duration(60)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("No se recibió ningún comando en el tiempo límite.")
                sub_voz.unregister()
                return 'finish'

            if self.comand == "trayectoria":
                rospy.loginfo("Comando 'trayectoria' recibido. Cambiando de estado.")
                sub_voz.unregister()
                return 'next_state'

            elif self.comand == "objetos":
                rospy.loginfo("Comando 'objetos' recibido. Navegando hacia objetos detectados.")
                sub_voz.unregister()
                return 'go_to_objects'

            elif self.comand == "finalizar":
                rospy.loginfo("Comando 'finalizar' recibido. Terminando misión.")
                sub_voz.unregister()
                return 'finish'

            rate.sleep()

    def command_callback(self, msg):
        self.comand = msg.data.strip().lower()


# Estado para la navegación hacia waypoints
class TurtleBot2NavigationState(State):
    def __init__(self, waypoints, max_attempts=3):
        State.__init__(self, outcomes=['route_completed', 'navigation_failed'])
        self.waypoints = waypoints
        self.max_attempts = max_attempts
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        rospy.loginfo("Conectando con move_base...")
        if not self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("No se pudo conectar con move_base.")
            return
        rospy.loginfo("Conexión con move_base establecida.")

        rospy.Subscriber('/detected_objects', PoseStamped, self.object_callback)

    def object_callback(self, msg):
        detected_objects.append({'x': msg.pose.position.x, 'y': msg.pose.position.y})

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación...")
        contador = 0
        for name, position in enumerate(self.waypoints):
            for attempt in range(1, self.max_attempts + 1):
                rospy.loginfo(f"Navegando hacia {name}, intento {attempt}.")
                goal = crear_destino(position[0], position[1])
                self.move_base_client.send_goal(goal)
                success = self.move_base_client.wait_for_result(rospy.Duration(120))
                if success:
                    rospy.loginfo(f"{name} alcanzado.")
                    break
                rospy.logwarn(f"Intento {attempt} fallido.")

                if attempt == self.max_attempts:
                    rospy.logerr(f"No se pudo alcanzar {name} en el último intento.")
                    if contador > 0:
                        previous_name, previous_position = self.waypoints[contador - 1]
                        rospy.loginfo(f"Intentando regresar al waypoint anterior: {previous_name}.")
                        
                        goal_prev = crear_destino(previous_position[0], previous_position[1])
                        self.move_base_client.send_goal(goal_prev)
                        success_prev = self.move_base_client.wait_for_result(rospy.Duration(120))
                        
                        if success_prev:
                            rospy.loginfo(f"Regresado exitosamente al waypoint anterior: {previous_name}.")
                            
                            rospy.loginfo(f"Intentando alcanzar nuevamente {name} desde {previous_name}.")
                            goal = crear_destino(position[0], position[1])
                            self.move_base_client.send_goal(goal)
                            success = self.move_base_client.wait_for_result(rospy.Duration(120))
                            
                            if success:
                                rospy.loginfo(f"{name} alcanzado desde el waypoint anterior.")
                                break
                            else:
                                rospy.logwarn(f"Fallo nuevamente al intentar alcanzar {name} desde {previous_name}.")
                        else:
                            rospy.logwarn(f"No se pudo regresar al waypoint anterior: {previous_name}.")
                    else:
                        rospy.logwarn("No existe un waypoint anterior para intentar una ruta alternativa.")
                
                contador = contador + 1

            if not success:
                rospy.logerr(f"Abortando misión: no se pudo alcanzar {name}.")
                return 'navigation_failed'
        
        if detected_objects:
            rospy.loginfo("Objetos detectados durante la ruta:")
            for i, obj in enumerate(detected_objects):
                rospy.loginfo(f"Objeto {i + 1}: x={obj['x']}, y={obj['y']}")
        else:
            rospy.loginfo("No se detectaron objetos durante la ruta.")
        
        return 'route_completed'


# Estado para navegar hacia objetos detectados
class GoToDetectedObjects(State):
    def __init__(self):
        State.__init__(self, outcomes=['objects_visited'])
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación hacia objetos detectados...")
        if not detected_objects:
            rospy.loginfo("No hay objetos detectados para visitar.")
            return 'objects_visited'

        for i, obj in enumerate(detected_objects):
            rospy.loginfo(f"Navegando hacia objeto {i + 1} en x={obj['x']}, y={obj['y']}.")
            goal = crear_destino(obj['x'], obj['y'])
            self.move_base_client.send_goal(goal)
            success = self.move_base_client.wait_for_result(rospy.Duration(60))
            if success:
                rospy.loginfo(f"Objeto {i + 1} alcanzado.")
            else:
                rospy.logwarn(f"No se pudo alcanzar el objeto {i + 1}.")
        return 'objects_visited'


def main():
    rospy.init_node('turtlebot2_navigation_mission')
    sm = StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:
        StateMachine.add('WAIT_ORDER', WaitForOrder(),
                         transitions={
                             'next_state': 'ROUTE_FOLLOWING',
                             'go_to_objects': 'GO_TO_OBJECTS',
                             'finish': 'mission_completed'
                         })

        StateMachine.add('ROUTE_FOLLOWING', TurtleBot2NavigationState(waypoints),
                         transitions={
                             'route_completed': 'WAIT_ORDER',
                             'navigation_failed': 'mission_failed'
                         })

        StateMachine.add('GO_TO_OBJECTS', GoToDetectedObjects(),
                         transitions={'objects_visited': 'WAIT_ORDER'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
