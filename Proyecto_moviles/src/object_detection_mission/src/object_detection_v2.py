#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smach import State,StateMachine
import smach_ros
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Para comandos de voz


# Definir waypoints de la ruta a seguir
waypoints = [
    ['1 Punto', (5.0, 4.0), (0.0, 0.0, 0.0, 1.0)],
    ['2 Punto', (8.0, 2.0), (0.0, 0.0, 0.0, 1.0)],
    ['3 Punto', (3.0, 4.0), (0.0, 0.0, 0.0, 1.0)]
    ]


class RouteFollowingState(State):
    def __init__(self, waypoints):
        State.__init__(self, 
                             outcomes=['route_completed', 'object_detected'],
                             output_keys=['detected_objects'])
        self.waypoints = waypoints
        self.detected_objects = []

    def execute(self, userdata):
        # Lógica para seguir la ruta cíclica
        for waypoint in self.waypoints:
            # Usar SimpleActionState para moverse a cada waypoint
            move_base_goal = self.create_move_base_goal(waypoint)
            
            # Realizar detección de objetos durante el movimiento
            object_detected = self.check_for_objects()
            
            if object_detected:
                # Guardar información del objeto
                detected_object = self.get_object_info()
                self.detected_objects.append(detected_object)
                
                # Pasar objetos detectados al siguiente estado
                userdata.detected_objects = self.detected_objects
                
                return 'object_detected'
        
        return 'route_completed'

    def create_move_base_goal(self, waypoint):
        # Crear goal para move_base
        goal = MoveBaseGoal()
        # Configurar coordenadas del waypoint
        return goal

    def check_for_objects(self):
        # Lógica de detección de objetos (imagen, láser, etc.)
        # Retorna True si se detecta un objeto, False en otro caso
        pass

    def get_object_info(self):
        # Obtener información del objeto detectado
        # Incluye posición, forma, etc.
        object_info = {
            'shape': '',  # 'triangle', 'rectangle', etc.
            'position': Pose2D(),  # Posición en el mapa
        }
        return object_info

class VoiceCommandState(State):
    def __init__(self):
        State.__init__(self, 
                             outcomes=['navigate_to_object', 'mission_complete'],
                             input_keys=['detected_objects'])
        self.voice_sub = None

    def execute(self, userdata):
        # Suscribirse a comandos de voz
        self.voice_sub = rospy.Subscriber('/voice_commands', String, self.voice_callback)
        
        # Esperar comando de voz
        # Dependiendo del comando, navegar al objeto específico
        
        return 'navigate_to_object'

    def voice_callback(self, msg):
        # Procesar comando de voz
        # Ejemplo: "Go to triangle" o "Go to rectangle"
        pass

class NavigateToObjectState(State):
    def __init__(self):
        State.__init__(self, 
                             outcomes=['object_reached', 'navigation_failed'],
                             input_keys=['detected_objects'])

    def execute(self, userdata):
        # Navegar al objeto específico seleccionado por comando de voz
        # Usar move_base para navegar a la posición del objeto
        
        return 'object_reached'

def main():
    rospy.init_node('mobile_robot_mission')
    
    # Crear máquina de estados
    sm = StateMachine(outcomes=['mission_completed'])
    
    with sm:
        # Añadir estados a la máquina
        StateMachine.add('ROUTE_FOLLOWING', 
                                RouteFollowingState(waypoints),
                                transitions={
                                    'route_completed': 'VOICE_COMMAND',
                                    'object_detected': 'ROUTE_FOLLOWING'
                                })
        
        StateMachine.add('VOICE_COMMAND', 
                                VoiceCommandState(),
                                transitions={
                                    'navigate_to_object': 'NAVIGATE_TO_OBJECT',
                                    'mission_complete': 'mission_completed'
                                })
        
        StateMachine.add('NAVIGATE_TO_OBJECT', 
                                NavigateToObjectState(),
                                transitions={
                                    'object_reached': 'VOICE_COMMAND',
                                    'navigation_failed': 'VOICE_COMMAND'
                                })
    
    # Declaraciones para ver de una forma visual la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')  
    sis.start()
    sm.execute()
    rospy.spin()   

if __name__ == '__main__':
    main()