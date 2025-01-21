#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
import pyttsx3

class InterfaceSelector:
    def __init__(self):
        rospy.init_node('interface_selector_node')
        self.engine = pyttsx3.init()
        self.selector_node = rospy.get_name()
        self.subVoice = rospy.Subscriber('voice_commands_persona', String, self.process_and_speak)
        self.interface_selected = False  # Bandera para verificar si ya se ha elegido una interfaz

        # Mensaje inicial
        self.speak_initial_message()

    def speak_initial_message(self):
        """Reproduce el mensaje inicial al inicio del nodo"""
        initial_message = "Selecciona un tipo de interfaz."
        rospy.loginfo(f"Mensaje inicial: {initial_message}")
        self.engine.say(initial_message)
        self.engine.runAndWait()
        self.engine.stop()

    def process_and_speak(self, msg):
        if self.interface_selected:
            return

        rospy.loginfo(f"Comando recibido: {msg.data}")
        response = ""

        if msg.data == "visual":
            response = "Iniciando visual."
        elif msg.data == "voz":
            response = "Iniciando voz."
        elif msg.data == "ambas":
            response = "Iniciando ambas."
        elif msg.data == "salir":
            response = "Finalizando la operación."

        rospy.loginfo(f"Respuesta generada: {response}")

        # Reproducir la respuesta
        self.engine.say(response)
        self.engine.runAndWait()
        self.engine.stop()

        # Controlar las interfaces según el comando
        self.control_interface(msg.data)

        # Marcar que se ha hecho la selección
        self.interface_selected = True
        
        # Finalizar este nodo
        self.kill_selector_node()

    def control_interface(self, command):
        """Controlar la interfaz según el comando recibido"""

        if command == "visual":
            rospy.loginfo("Iniciando visual.")
            self.kill_voice_processing_node()
            os.system("roslaunch object_detection_mission visual.launch")
        elif command == "voz":
            rospy.loginfo("Iniciando voz")
            os.system("roslaunch object_detection_mission voz.launch")
        elif command == "ambas":
            rospy.loginfo("Iniciando ambas")
            os.system("roslaunch object_detection_mission todo.launch")
        elif command == "salir":
            rospy.loginfo("Finalizando todos los nodos...")
            os.system("rosnode kill -a")  # Detener todos los nodos
        else:
            rospy.logwarn("Comando no reconocido.")

    def kill_selector_node(self):
        """Finalizar este nodo una vez elegida la interfaz"""
        rospy.loginfo("Finalizando nodo selector.")
        os.system(f"rosnode kill {self.selector_node}")

    def kill_voice_processing_node(self):
        """Detener nodos relacionados con el procesamiento de voz"""
        rospy.loginfo("Deteniendo nodos de voz...")
        os.system("rosnode kill /escuhar")  # Nodo de procesamiento de voz
        #os.system("rosnode kill /ubicacion")  # Nodo de ubicación (si corresponde)

    def listen_and_publish(self):
        rospy.loginfo("Esperando comandos de voz para elegir interfaz...")
        rospy.spin()

if __name__ == '__main__':
    try:
        interface_selector = InterfaceSelector()
        interface_selector.listen_and_publish()
    except rospy.ROSInterruptException:
        pass
