#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

# Puntos para las trayectorias del mapa
waypoints = [
    ['Base', (5.0, 4.0), (0.0, 0.0, 0.0, 1.0)]  # Localización en el mapa de la base (x=5, y=4, orientación cualquiera)
]

class ObjectDetectionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_detected', 'mission_complete'])
        self.detected_objects = []  # Lista para almacenar objetos
        self.image_sub = None
        self.current_pose = None

    def execute(self, userdata):
        # Lógica de exploración y detección de objetos
        # Suscribirse a tópicos de imagen y odometría
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.process_image)
        
        # Implementar lógica de movimiento exploratorio
        while not rospy.is_shutdown():
            if self.object_detected:
                return 'object_detected'
            
            # Lógica de movimiento exploratorio
            # Usar SimpleActionState para moverse a puntos de interés
        
        return 'mission_complete'

    def process_image(self, msg):
        # Procesar imagen para detectar formas
        # Implementar algoritmo de detección de formas
        # Si se detecta un objeto, guardar su información
        pass

    def register_object(self, shape, pose):
        # Método para registrar objetos detectados
        object_info = {
            'shape': shape,
            'pose': pose
        }
        self.detected_objects.append(object_info)

class RegisterObjectState(smach.State):
    def __init__(self, object_detection_state):
        smach.State.__init__(self, outcomes=['registered', 'failed'])
        self.object_detection_state = object_detection_state

    def execute(self, userdata):
        # Obtener el último objeto detectado
        if self.object_detection_state.detected_objects:
            last_object = self.object_detection_state.detected_objects[-1]
            
            # Lógica para guardar coordenadas y forma
            rospy.loginfo(f"Objeto registrado: {last_object}")
            return 'registered'
        
        return 'failed'

class ReturnBaseState(smach.State):
    def __init__(self, waypoints):
        smach.State.__init__(self, outcomes=['succeed', 'aborted'])
        self.waypoints = waypoints

    def execute(self, userdata):
        # Lógica para volver a la base similar al código anterior
        move_base_action = smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=self.get_base_goal())
        result = move_base_action.execute(userdata)

        if result == 'succeed':
            rospy.loginfo("Robot ha regresado a la base")
            return 'succeed'
        
        return 'aborted'

    def get_base_goal(self):
        # Lógica para obtener coordenadas de la base
        base_goal = MoveBaseGoal()
        # Configurar coordenadas de la base
        return base_goal

def main():
    rospy.init_node('object_detection_mission')
    
    # Crear máquina de estados
    sm = smach.StateMachine(outcomes=['mission_completed'])
    
    # Instancia de estado de detección de objetos
    object_detection_state = ObjectDetectionState()
    
    with sm:
        smach.StateMachine.add('OBJECT_DETECTION', object_detection_state, 
                                transitions={
                                    'object_detected':'REGISTER_OBJECT', 
                                    'mission_complete':'RETURN_BASE'
                                })
        
        smach.StateMachine.add('REGISTER_OBJECT', RegisterObjectState(object_detection_state),
                                transitions={
                                    'registered':'OBJECT_DETECTION',
                                    'failed':'RETURN_BASE'
                                })
        
        smach.StateMachine.add('RETURN_BASE', ReturnBaseState(waypoints),
                                transitions={
                                    'succeed':'mission_completed',
                                    'aborted':'mission_completed'
                                })
    
    # Ejecutar máquina de estados
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()