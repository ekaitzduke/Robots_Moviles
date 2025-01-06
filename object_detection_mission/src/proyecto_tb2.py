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
import random

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

# Función global para crear el movimiento al destino indicado con un rango de error (Ya que en dicha posición se encontrará un objeto)
def crear_destino_con_rango(x, y, rango=0.5):
    """
    Crea un objetivo de navegación para move_base dentro de un rango especificado.
    """
    dx = random.uniform(-rango, rango)  # Variación aleatoria en x
    dy = random.uniform(-rango, rango)  # Variación aleatoria en y

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x + dx
    goal.target_pose.pose.position.y = y + dy
    goal.target_pose.pose.orientation.w = 1.0  # Orientación neutral
    return goal


# Estado para esperar comandos de voz
class WaitForOrder(State):
    def __init__(self):
        State.__init__(self, outcomes=['trayectory', 'go_to_objects', 'patrol_objects', 'finish'])
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
                return 'trayectory'

            elif self.comand == "objetos":
                rospy.loginfo("Comando 'objetos' recibido. Navegando hacia objetos detectados.")
                sub_voz.unregister()
                return 'go_to_objects'
            
            elif self.comand == "patrulla":
                rospy.loginfo("Comando 'patrulla' recibido. Patrullando los objetos.")
                return 'patrol_objects'

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

        # Subscripción al tópico de deteción de objetos
        rospy.Subscriber('/detected_objects', PoseStamped, self.object_callback)

    def object_callback(self, msg):
        detected_objects.append({'x': msg.pose.position.x, 'y': msg.pose.position.y})

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación...")
        for name, position in self.waypoints:
            for attempt in range(1, self.max_attempts + 1):
                rospy.loginfo(f"Navegando hacia {name}, intento {attempt}.")
                goal = crear_destino(position[0], position[1])
                self.move_base_client.send_goal(goal)
                success = self.move_base_client.wait_for_result(rospy.Duration(120))
                if success:
                    rospy.loginfo(f"{name} alcanzado.")
                    break
                rospy.logwarn(f"Intento {attempt} fallido.")
            else:
                rospy.logerr(f"No se pudo alcanzar {name}.")
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
    def __init__(self, waypoints):
        State.__init__(self, outcomes=['objects_visited'])
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.waypoints = waypoints

    def execute(self, userdata):
        rospy.loginfo("Iniciando navegación hacia objetos detectados...")

        # Comprobación de objetos detectados
        if not detected_objects:
            rospy.loginfo("No hay objetos detectados para visitar.")
            return 'objects_visited'

        # Bucle para recorrer los objetos
        for i, obj in enumerate(detected_objects):
            rospy.loginfo(f"Navegando hacia objeto {i + 1} en x={obj['x']}, y={obj['y']}.")
            goal = crear_destino_con_rango(obj['x'], obj['y'])
            self.move_base_client.send_goal(goal)
            success = self.move_base_client.wait_for_result(rospy.Duration(60))
            if success:
                rospy.loginfo(f"Objeto {i + 1} alcanzado.")
            else:
                rospy.logwarn(f"No se pudo alcanzar el objeto {i + 1}.")

        # Una vez se ha salido de la patrulla se vuelve a la posición de Inicio
        rospy.loginfo("Regresando a la posición de Inicio")
        goal = crear_destino(self.waypoints[0][1][0], self.waypoints[0][1][1])
        self.move_base_client.send_goal(goal)

        # Verificar si el objetivo fue alcanzado
        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Vuelta al Inicio.")
        else:
            rospy.logwarn("No se pudo alcanzar el objetivo.")
        
        return 'objects_visited'

# Estado para hacer como una patrulla al rededor de los objetos
class PatrolAroundObjects(State):
    def __init__(self, waypoints):
        State.__init__(self, outcomes=['patrol_finished'])
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Variables para el estado
        self.completed = False
        self.comand = None 
        self.waypoints = waypoints

        # Puede que no haga falta. Comprobarlo
        rospy.Subscriber('/voice_commands', String, self.command_callback)      # Subscripción al tópic de los comandos de voz
    
    def execute(self, userdata):
        rospy.loginfo("Iniciando la patrulla al rededor de los objetos")

        # Comprobación de objetos detectados
        if not detected_objects:
            rospy.loginfo("No hay objetos detectados para patrullar.")
            return 'patrol_finished'

        current_index = 0   # Índice del primer objeto
        # Bucle while para realizar la patrulla
        while not self.completed and not rospy.is_shutdown():
            rospy.loginfo("Patrullando hacia el siguiente objeto...")

            obj = detected_objects[current_index]
            goal = crear_destino_con_rango(obj['x'], obj['y'])
            self.move_base_client.send_goal(goal)

            if self.move_base_client.wait_for_result(rospy.Duration(60)):
                rospy.loginfo(f"Objeto en x={obj['x']}, y={obj['y']} alcanzado.")
                current_index = current_index + 1
                if current_index == len(detected_objects):
                    current_index = 0
            else:
                rospy.logwarn(f"No se pudo alcanzar el objeto en x={obj['x']}, y={obj['y']}.")

            # Verificar si se recibió el comando de finalizar
            if self.completed:
                rospy.loginfo("Finalizando patrullaje por comando de voz.")
                break

        # Una vez se ha salido de la patrulla se vuelve a la posición de Inicio
        rospy.loginfo("Regresando a la posición de Inicio")
        goal = crear_destino(self.waypoints[0][1][0], self.waypoints[0][1][1])
        self.move_base_client.send_goal(goal)

        # Verificar si el objetivo fue alcanzado
        state = self.move_base_client.get_state()
        self.move_base_client.send_goal(goal)
        if self.move_base_client.wait_for_result(rospy.Duration(60)):
            rospy.loginfo("Vuelta al Inicio completada.")
        else:
            rospy.logwarn("No se pudo alcanzar el objetivo de Inicio.")
            
        return 'patrol_finished'

    def command_callback(self, msg):
        self.comand = msg.data.strip().lower()

        # Si se recibe el comando "finalizar" se sale del bucle de la patrulla
        if self.comand == "finalizar":
            self.completed = True

# Main del programa
def main():
    rospy.init_node('turtlebot2_navigation_mission')
    sm = StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:
        StateMachine.add('WAIT_ORDER', WaitForOrder(),
                         transitions={
                             'trayectory': 'ROUTE_FOLLOWING',
                             'go_to_objects': 'GO_TO_OBJECTS',
                             'patrol_objects': 'PATROL_OBJECTS',
                             'finish': 'mission_completed'
                         })

        StateMachine.add('ROUTE_FOLLOWING', TurtleBot2NavigationState(waypoints),
                         transitions={
                             'route_completed': 'WAIT_ORDER',
                             'navigation_failed': 'mission_failed'
                         })

        StateMachine.add('GO_TO_OBJECTS', GoToDetectedObjects(waypoints),
                         transitions={'objects_visited': 'WAIT_ORDER'})
        
        StateMachine.add('PATROL_OBJECTS', PatrolAroundObjects(waypoints),
                         transitions={'patrol_finished': 'WAIT_ORDER'})

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
