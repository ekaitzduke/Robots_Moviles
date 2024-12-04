#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import difflib                          # Librería para el procesado de las palabras
from vosk import Model, KaldiRecognizer  # Librerías de Vosk para reconocimiento de voz
import wave
import pyaudio

# Lista de comandos válidos
VALID_COMMANDS = ["adelante", "atrás", "izquierda", "derecha", "detener", "cocina", "comedor", "habitación", "finalizar", "visual", "voz", "ambas", "reanudar"]

# Compara el texto (comandos de voz) con la lista de comandos
def normalize_command(command):
    command = command.lower()
    close_matches = difflib.get_close_matches(command, VALID_COMMANDS, n=1, cutoff=0.7)
    return close_matches[0] if close_matches else None

# Función para escuchar y publicar
def listen_and_publish():
    rospy.init_node('speech_to_text_node')
    pub = rospy.Publisher('voice_commands_persona', String, queue_size=10)

    # Carga el modelo de Vosk
    rospy.loginfo("Cargando modelo de Vosk...")
    model = Model("/home/franciscocj/catkin_ws/src/persona_robot_pkg/model/vosk-model-small-es-0.42")  # Asegúrate de descargar un modelo de Vosk y colocarlo en un directorio llamado 'model'

    recognizer = KaldiRecognizer(model, 16000)

    # Configuración del micrófono
    audio = pyaudio.PyAudio()
    stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
    stream.start_stream()

    rospy.loginfo("Nodo de reconocimiento de voz listo. Escuchando...")
    while not rospy.is_shutdown():
        try:
            data = stream.read(4096, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                result = recognizer.Result()
                text = eval(result).get("text", "")  # Extrae el texto del reconocimiento
                rospy.loginfo(f"Comando de voz reconocido: {text}")

                # Normalización del comando de voz
                command_normalized = normalize_command(text)
                if command_normalized:
                    pub.publish(command_normalized)
                    rospy.loginfo(f"Comando de voz publicado: {command_normalized}")
                else:
                    rospy.logwarn("Comando no válido.")
        except Exception as e:
            rospy.logerr(f"Error en el reconocimiento de voz: {e}")

if __name__ == '__main__':
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass