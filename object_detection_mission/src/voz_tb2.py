#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import difflib
from vosk import Model, KaldiRecognizer
import wave
import pyaudio
import rospkg
import os

# Lista de comandos válidos para TurtleBot 2
VALID_COMMANDS = [
    "trayectoria", "objetos", "patrulla", "finalizar"
]

def normalize_command(command):
    """
    Normaliza el comando de voz comparándolo con la lista de comandos válidos
    """
    command = command.lower()
    close_matches = difflib.get_close_matches(command, VALID_COMMANDS, n=1, cutoff=0.7)
    return close_matches[0] if close_matches else None

def listen_and_publish():
    """
    Nodo de reconocimiento de voz para TurtleBot 2
    """
    # Inicializar nodo ROS
    rospy.init_node('turtlebot2_voice_control', anonymous=True)
    
    # Publicador de comandos de voz
    pub = rospy.Publisher('/voice_commands', String, queue_size=10)
    
    # Ruta al modelo de Vosk (ajustar según tu configuración)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('object_detection_mission')

    vosk_model_path = os.path.join(package_path, 'model', 'vosk-model-small-es-0.42')
    
    # Verificar existencia del modelo
    if not os.path.exists(vosk_model_path):
        rospy.logerr(f"Modelo de Vosk no encontrado en: {vosk_model_path}")
        rospy.logerr("Por favor, descarga el modelo de Vosk para español")
        return
    
    # Cargar modelo de Vosk
    rospy.loginfo("Cargando modelo de Vosk para TurtleBot 2...")
    try:
        model = Model(vosk_model_path)
        recognizer = KaldiRecognizer(model, 16000)
    except Exception as e:
        rospy.logerr(f"Error al cargar el modelo Vosk: {e}")
        return
    
    # Configuración del micrófono
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16, 
        channels=1, 
        rate=16000, 
        input=True, 
        frames_per_buffer=8192
    )
    stream.start_stream()
    
    rospy.loginfo("TurtleBot 2 - Nodo de control por voz iniciado")
    
    # Ciclo principal de reconocimiento
    try:
        while not rospy.is_shutdown():
            try:
                data = stream.read(4096, exception_on_overflow=False)
                if recognizer.AcceptWaveform(data):
                    result = recognizer.Result()
                    text = eval(result).get("text", "")
                    rospy.loginfo(f"Comando reconocido: {text}")
                    
                    # Normalizar y publicar comando
                    command_normalized = normalize_command(text)
                    if command_normalized:
                        pub.publish(command_normalized)
                        rospy.loginfo(f"Comando publicado: {command_normalized}")
                    else:
                        rospy.logwarn("Comando no reconocido")
            
            except Exception as e:
                rospy.logerr(f"Error en reconocimiento de voz: {e}")
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de control por voz detenido")
    
    finally:
        # Limpiar recursos
        stream.stop_stream()
        stream.close()
        audio.terminate()

def main():
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
