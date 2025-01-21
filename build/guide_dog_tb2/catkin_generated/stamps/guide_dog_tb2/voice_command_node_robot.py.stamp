#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import pyttsx3

# Variable global que indica si el robot está en movimiento
en_movimiento = False

def actualizar_estado_movimiento(msg):
    global en_movimiento
    en_movimiento = msg.data  # True si está en movimiento, False si está detenido


def process_and_speak(msg):
    global en_movimiento
    rospy.loginfo(f"Comando recibido: {msg.data}")
    
    if en_movimiento and msg.data in ["electrónica", "aula", "laboratorio", "almacén", "pasillo", "servicio", "salida"]:
        rospy.loginfo(f"El robot ya está en movimiento, ignorando el comando de destino: {msg.data}")
        return

    response = ""

    # Genera una respuesta basada en el comando
    if msg.data == "adelante":
        response = "Avanzando hacia adelante."
    elif msg.data == "atrás":
        response = "Avanzando hacia atras."
    elif msg.data == "izquierda":
        response = "Girando a la izquierda."
    elif msg.data == "derecha":
        response = "Girando a la derecha."
    elif msg.data == "detener":
        response = "Detenido."
    elif msg.data == "electrónica":
        response = "Dirigiéndome a electronica."
    elif msg.data == "aula":
        response = "Dirigiéndome al aula."
    elif msg.data == "laboratorio":
        response = "Dirigiéndome al laboratorio."
    elif msg.data == "almacén":
        response = "Dirigiéndome al almacen."
    elif msg.data == "pasillo":
        response = "Dirigiéndome al pasillo."
    elif msg.data == "servicio":
        response = "Dirigiéndome al servicio."
    elif msg.data == "salida":
        response = "Dirigiéndome a la salida."
    elif msg.data == "reanudar":
        response = "Reanudando la marcha"
    elif msg.data == "finalizar":
        response = "Finalizando operacion. Hasta luego."
    else:
        response = "No entendí el comando. Por favor repitelo."

    rospy.loginfo(f"Respuesta generada: {response}")

    # Reproducir la respuesta con pyttsx3
    if response:
        engine = pyttsx3.init()
        engine.say(response)
        engine.runAndWait()
        engine.stop()

def configure_engine():
    global engine
    engine = pyttsx3.init()

    # Ajustar la velocidad (rate) a un valor más lento
    rate = engine.getProperty('rate')  # Obtiene el valor actual
    rospy.loginfo(f"Velocidad actual de síntesis: {rate}")
    engine.setProperty('rate', 220)  # Reduce la velocidad a 150 palabras por minuto

    # Ajustar el volumen
    volume = engine.getProperty('volume')  # Obtiene el volumen actual
    rospy.loginfo(f"Volumen actual de síntesis: {volume}")
    engine.setProperty('volume', 0.9)  # Establece el volumen al 90%

    # Cambiar la voz (elige una más natural si está disponible)
    voices = engine.getProperty('voices')
    rospy.loginfo("Voces disponibles:")
    for idx, voice in enumerate(voices):
        rospy.loginfo(f"{idx}: {voice.name} ({voice.languages})")
    engine.setProperty('voice', voices[26].id)  # Cambia a la primera voz disponible (ajusta según preferencia)

def command_processor_and_speaker():
    rospy.init_node('command_processor_and_speaker_node')
    configure_engine()
    rospy.Subscriber('voice_commands_persona', String, process_and_speak)
    rospy.loginfo("Nodo combinado de procesamiento y síntesis listo.")
    rospy.Subscriber('/robot_en_movimiento', Bool, actualizar_estado_movimiento)  # Se suscribe al estado de movimiento
    rospy.spin()
    engine.stop()  # Detener el motor al finalizar

if __name__ == '__main__':
    try:
        command_processor_and_speaker()
    except rospy.ROSInterruptException:
        pass
