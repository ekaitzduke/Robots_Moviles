#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from smach import State, StateMachine
from smach_ros import IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from tf.transformations import quaternion_from_euler
import threading
import subprocess

# Variable global para el destino
global_goal = None

# Publicador para los comandos de movimiento directo
pub = None

# Publicador para indicar si el robot está en movimiento
pub_en_movimiento = None  

# Publicador para no hablar durante la ruta
pub_ruta = None
complex = False

# Variables para reanudar la marcha
activate_detner = False
last_move = None

# Variable global para almacenar el último comando Twist
current_twist = None
stop_movement = False

# Procesa comandos de voz para movimiento directo
def procesar_comando_voz(comando):
    global current_twist, stop_movement

    twist = Twist()
    stop_movement = False  # Reiniciar stop_movement para aceptar nuevos comandos

    if comando == "adelante":
        twist.linear.x = 0.2
    elif comando == "atrás":
        twist.linear.x = -0.2
    elif comando == "izquierda":
        twist.angular.z = 0.5
    elif comando == "derecha":
        twist.angular.z = -0.5
    elif comando == "detener":
        rospy.loginfo("Deteniendo el robot desde procesar_comando_voz.")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        stop_movement = True  # Detener el movimiento (pero no la escucha de comandos de voz)
        current_twist = None  # Eliminar el comando actual
        pub.publish(twist)  # Publicar twist vacío para detener el robot
        return None  # No devolver twist, ya que no se moverá
    else:
        return None  # Comando no reconocido para movimiento directo

    current_twist = twist  # Guardar el último comando de movimiento
    return twist

def publicar_movimiento():
    global current_twist, stop_movement

    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)
    while not rospy.is_shutdown():
        if stop_movement:
            # Evitar la publicación de twists si stop_movement está activo
            current_twist = None  # Asegurarse de que no se reutilice el último comando
        elif current_twist:
            pub.publish(current_twist)  # Publicar el último comando de movimiento

        rate.sleep()


# Clase de estado para esperar un destino
class EsperarDestino(State):
    def __init__(self):
        State.__init__(self, outcomes=['nuevo_destino', 'basics', 'finalizar'])
        self.movimiento_activo = False  # Indica si se está en movimiento básico

    def execute(self, userdata):
        rospy.loginfo("Esperando un comando de voz para definir el destino...")

        pub_en_movimiento.publish(False)  # Aquí no estamos en movimiento

        global activate_detner, last_move

        # No se esta realizando un movimiento complejo
        global complex
        complex = False
        pub_ruta.publish(complex)

        global global_goal

        while not rospy.is_shutdown():
            comando = rospy.wait_for_message('voice_commands_persona', String).data.strip().lower()

            if comando == "finalizar":
                rospy.loginfo("Fin del programa.")
                return 'finalizar'

            elif comando == "reanudar" and activate_detner:
                global_goal = last_move
                last_move = None
                activate_detner = False
                return 'nuevo_destino'

            # Si se están ejecutando comandos básicos
            if comando in ["adelante", "atrás", "izquierda", "derecha"]:
                rospy.loginfo(f"Comando básico '{comando}' recibido.")
                self.movimiento_activo = True

                pub_en_movimiento.publish(True)  # Indicar que el robot está en movimiento
                
                twist = procesar_comando_voz(comando)
                if twist:
                    pub.publish(twist)
                continue  # No salir del bucle aún

            elif comando == "detener":
                rospy.loginfo("Comando 'detener' recibido. Movimiento básico detenido.")
                self.movimiento_activo = False

                pub_en_movimiento.publish(False)  # Indicar que el robot no está en movimiento

                twist = Twist()  # Twist vacío para detener el robot
                pub.publish(twist)  # Publicar el twist vacío

                global stop_movement, current_twist
                stop_movement = True  # Indicar que se detenga la publicación de movimientos
                current_twist = None  # Asegurarse de que no se reutilice el último comando
                continue  # Mantenerse en el ciclo, listo para recibir nuevos comandos


            # Si el robot no está en movimiento básico, aceptar comandos de destino
            if not self.movimiento_activo and comando in ["electrónica", "aula", "laboratorio", "almacén", "pasillo", "servicio", "salida"]:
                rospy.loginfo(f"Comando '{comando}' recibido. El robot se moverá a {comando}.")
                coordenadas = {
                    "electrónica": (-4.6, 4.5, 1.57),
                    "aula": (0, 3, 1.57),
                    "laboratorio": (3.0, -3.0, 1.57),
                    "almacén": (4.4, 6.0, 0),
                    "pasillo": (-2.0, -5.6, 0),
                    "servicio": (-10.5, -13, 0),
                    "salida": (14.6, -5, 0)
                }

                if activate_detner: 
                    activate_detner = False

                x, y, yaw = coordenadas[comando]
                global_goal = self.crear_destino(x, y, yaw)
                return "nuevo_destino"

            rospy.loginfo("Comando no válido o no permitido en este estado.")
            continue

    def crear_destino(self, x, y, yaw=0.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal


# Clase de estado final
class Finalizado(State):
    def __init__(self):
        State.__init__(self, outcomes=['end'])

    def execute(self, userdata):
        # Matar todos los nodos
        rospy.loginfo("Finalizando. Mataré todos los nodos ejecutados.")
        
        try:
            # Obtener la lista de nodos activos
            active_nodes = subprocess.check_output(["rosnode", "list"]).splitlines()

            for node in active_nodes:
                # Matar cada nodo de manera explícita
                subprocess.call(["rosnode", "kill", node])

            rospy.loginfo("Todos los nodos han sido detenidos.")
        
        except Exception as e:
            rospy.logerr(f"Error al finalizar: {e}")
        
        return 'end'


# Clase de estado para manejar los movimientos básicos
class MovimientoBasico(State):
    def __init__(self):
        State.__init__(self, outcomes=['detener', 'finalizar'])

    def execute(self, userdata):
        rospy.loginfo("El robot está listo para movimientos básicos. Envíe 'detener' para finalizar.")
        
        pub_en_movimiento.publish(True)  # Ahora está en movimiento

        while not rospy.is_shutdown():
            comando = rospy.wait_for_message('voice_commands_persona', String).data.strip().lower()
            if comando == 'detener':
                # Publicar Twist con velocidades en cero para detener el robot
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                rospy.loginfo("Robot detenido.")

                pub_en_movimiento.publish(False)  # Se detuvo

                return 'detener'
            elif comando == 'finalizar':
                rospy.loginfo("Finalizando el programa desde movimientos básicos.")
                return 'finalizar'
            else:
                # Procesar otros comandos de movimiento básico
                twist = procesar_comando_voz(comando)
                if twist:
                    pub.publish(twist)


# Clase de estado para mover el robot al destino
class MoverRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['detener', 'completado', 'fallido'])

    def execute(self, userdata):
        rospy.loginfo("El robot se está moviendo al destino.")

        pub_en_movimiento.publish(True)  # Ahora está en movimiento

        global complex
        complex = True
        pub_ruta.publish(complex)    

        global activate_detner, last_move

        global global_goal
        # Cliente para la acción de move_base
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        # Enviar el destino al cliente de acción
        goal = global_goal

        if activate_detner is False:
            last_move = goal 

        move_base_client.send_goal(goal)

        # Mientras el robot esté en movimiento, escuchar comandos de voz
        while not rospy.is_shutdown():
            # Verificar el estado del cliente
            state = move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Destino alcanzado con éxito.")
                complex = False
                pub_ruta.publish(complex)

                pub_en_movimiento.publish(False)  # Se detuvo

                return 'completado'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.loginfo("No se pudo alcanzar el destino.")
                complex = False
                pub_ruta.publish(complex)
                return 'fallido'

            # Escuchar comandos de voz
            try:
                comando = rospy.wait_for_message('voice_commands_persona', String, timeout=0.5).data.strip().lower()
                if comando == 'detener':
                    rospy.loginfo("Comando 'detener' recibido. Cancelando el objetivo.")
                    move_base_client.cancel_goal()  # Se cancela el movimiento del SimpleActionState
                    complex = False
                    activate_detner = True
                    pub_ruta.publish(complex)
                    return 'detener'
            except rospy.ROSException:
                pass  # No se recibió ningún mensaje en el tiempo de espera

        rospy.loginfo("El robot finalizó el estado de mover robot.")
        return 'fallido'


if __name__ == '__main__':
    rospy.init_node("robot_control_por_voz")

    # Publicador de movimiento directo
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # Publicador de movimiento complejo
    pub_ruta = rospy.Publisher('complex', Bool, queue_size=10)  

    # Inicia el hilo de publicación continua
    movimiento_thread = threading.Thread(target=publicar_movimiento)
    movimiento_thread.daemon = True
    movimiento_thread.start()

    # Publicador de estado de movimiento
    pub_en_movimiento = rospy.Publisher('/robot_en_movimiento', Bool, queue_size=10)

    # Crear la máquina de estados
    sm = StateMachine(outcomes=['end'])

    with sm:
        # Estado: EsperarDestino
        StateMachine.add(
            'EsperarDestino',
            EsperarDestino(),
            transitions={'nuevo_destino': 'MoverRobot', 'basics': 'MovimientoBasico', 'finalizar': 'Finalizado'}
        )

        # Estado: MoverRobot
        StateMachine.add(
            'MoverRobot',
            MoverRobot(),
            transitions={'detener': 'EsperarDestino', 'completado': 'EsperarDestino', 'fallido': 'EsperarDestino'}
        )

        # Estado: MovimientoBasico
        StateMachine.add(
            'MovimientoBasico',
            MovimientoBasico(),
            transitions={'detener': 'EsperarDestino', 'finalizar': 'Finalizado'}
        )

        # Estado final: Finalizado
        StateMachine.add(
            'Finalizado',
            Finalizado(),
            transitions={'end': 'end'}
        )

    # Servidor de introspección para SMACH
    sis = IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("Nodo de control por voz iniciado.")
    sm.execute()
    rospy.spin()
