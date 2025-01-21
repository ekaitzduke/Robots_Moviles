#!/usr/bin/env python3
import rospy
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
from PIL import ImageDraw
from std_msgs.msg import String
import cv2
import rospkg
import os

class RosTkinterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Teleoperation Interface")
        self.root.geometry("1770x500")

        self.create_image_widgets()

        # Inicializar ROS node
        rospy.init_node('robot_teleoperation', anonymous=True)
        
        # Publicador de ROS para comandos de movimiento y comandos de voz
        self.pub_movement = rospy.Publisher('robot_movement', String, queue_size=10)
        self.pub_voice = rospy.Publisher('voice_commands_persona', String, queue_size=10)
        
        # Suscriptor para el estado del robot
        rospy.Subscriber('voice_commands_persona', String, self.update_robot_status)

        # Suscriptor para las imágenes de la cámara
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.update_camera_image)

        # Inicializar CvBridge
        self.bridge = CvBridge()

        # Crear elementos de la interfaz
        self.create_widgets()

        # Agregar la imagen para obtener coordenadas
        self.create_image_widgets()
    
    def dibujar_rectangulos(self):
        # Crear un objeto de dibujo sobre una imagen con canal alfa
        overlay = PILImage.new("RGBA", self.image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)

        # Definir las áreas clicables, sus etiquetas y colores
        clickable_areas = {
            "aula": {"coords": (440, 119, 486, 159), "color": (0, 255, 0, 128)},  # Verde semitransparente
            "electrónica": {"coords": (350, 70, 382, 106), "color": (0, 0, 255, 128)},  # Azul semitransparente
            "almacén": {"coords": (532, 75, 566, 105), "color": (255, 255, 0, 128)},  # Amarillo semitransparente
            "laboratorio": {"coords": (397, 223, 563, 312), "color": (255, 0, 0, 128)},  # Rojo semitransparente
            "pasillo": {"coords": (352, 270, 397, 316), "color": (128, 0, 128, 128)},  # Púrpura semitransparente
            "servicio": {"coords": (145, 393, 206, 423), "color": (0, 255, 255, 128)},  # Cian semitransparente
            "salida": {"coords": (671, 341, 732, 365), "color": (255, 165, 0, 128)},  # Naranja semitransparente
        }

        # Dibujar rectángulos con colores específicos
        for area, props in clickable_areas.items():
            coords = props["coords"]
            color = props["color"]
            draw.rectangle(coords, fill=color, outline="black", width=2)  # Fondo semitransparente y borde negro

        # Combinar la superposición con la imagen original
        self.image = PILImage.alpha_composite(self.image.convert("RGBA"), overlay).convert("RGB")



    def create_widgets(self):
        label_font = ("Arial", 12)
        button_font = ("Arial", 12)
        entry_width = 15

        # Campos de coordenadas
        coord_frame = tk.Frame(self.root)
        coord_frame.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        tk.Button(coord_frame, text="Enviar Coordenadas", font=button_font, command=self.send_coordinates).grid(row=0, column=4, padx=5, pady=5)

        # Menú desplegable de habitaciones y otros lugares
        self.room_selector = ttk.Combobox(coord_frame, values=["electrónica", "aula", "laboratorio", "almacén", "pasillo", "servicio", "salida"], state="readonly", width=entry_width)
        self.room_selector.grid(row=0, column=0, padx=5, pady=5)
        self.room_selector.set("Seleccionar destino")

        # Botones para detener y reanudar el robot
        button_frame = tk.Frame(self.root)
        button_frame.grid(row=1, column=0, padx=20, pady=10, sticky="w")

        tk.Button(button_frame, text="Parar el robot", bg="red", fg="white", font=button_font, command=self.stop_robot).grid(row=0, column=0, padx=10, pady=5)
        tk.Button(button_frame, text="Reanudar el robot", bg="green", fg="white", font=button_font, command=self.resume_robot).grid(row=0, column=1, padx=10, pady=5)

        # Área de visualización del estado del robot
        self.status_label = tk.Label(self.root, text="Robot moviéndose", font=("Arial", 14), borderwidth=2, relief="solid", width=20, height=2)
        self.status_label.grid(row=2, column=0, padx=20, pady=10, sticky="w")

        # Flechas para el movimiento
        control_frame = tk.Frame(self.root)
        control_frame.grid(row=4, column=0, padx=20, pady=10, sticky="w")

        tk.Button(control_frame, text="↑", font=button_font, width=5, command=self.move_forward).grid(row=0, column=1, padx=10, pady=5)
        tk.Button(control_frame, text="←", font=button_font, width=5, command=self.turn_left).grid(row=1, column=0, padx=10, pady=5)
        tk.Button(control_frame, text="↓", font=button_font, width=5, command=self.move_backward).grid(row=1, column=1, padx=10, pady=5)
        tk.Button(control_frame, text="→", font=button_font, width=5, command=self.turn_right).grid(row=1, column=2, padx=10, pady=5)

        # Ajustar la imagen de la cámara
        self.camera_label = tk.Label(self.root)
        self.camera_label.grid(row=0, column=5, rowspan=6, padx=10, pady=10)  # Cambié el índice de columna a 5

    def create_image_widgets(self):
        # Obtener la ruta al paquete object_detection_mission
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detection_mission')

        # Crear la ruta completa al archivo map.jpg
        image_path = os.path.join(package_path, 'worlds', 'map2.jpg')

        # Cargar la imagen para que self.image esté disponible
        self.image = PILImage.open(image_path)  # ✅ Definir self.image antes de usarla en dibujar_rectangulos


        # Dibujar rectángulos en la imagen
        self.dibujar_rectangulos()

        # Convertir la imagen a un formato compatible con Tkinter
        self.image_tk = ImageTk.PhotoImage(self.image)

        # Crear un widget Label para mostrar la imagen
        self.label = tk.Label(self.root, image=self.image_tk)
        self.label.grid(row=0, column=6, rowspan=6, padx=10, pady=10)

        # Enlazar el clic izquierdo en la imagen a la función obtener_coordenadas
        self.label.bind("<Button-1>", self.obtener_coordenadas)

    def obtener_coordenadas(self, event):
        # Obtener las coordenadas relativas al Frame de la ventana pequeña
        x, y = event.x, event.y

        # Determinar qué habitación corresponde a las coordenadas
        if 440 <= x <= 486 and 119 <= y <= 159:
            self.send_command("aula")
        elif 350 <= x <= 382 and 70 <= y <= 106:
            self.send_command("electrónica")
        elif 532 <= x <= 566 and 75 <= y <= 105:
            self.send_command("almacén")
        elif 397 <= x <= 563 and 223 <= y <= 312:
            self.send_command("laboratorio")
        elif 352 <= x <= 397 and 270 <= y <= 316:
            self.send_command("pasillo")
        elif 145 <= x <= 206 and 393 <= y <= 423:
            self.send_command("servicio")
        elif 671 <= x <= 732 and 341 <= y <= 732:
            self.send_command("salida")
        else:
            print(f"Coordenadas del clic: ({x}, {y}) - Sin habitación asociada")

    # Función para recibir y mostrar el estado del robot
    def update_robot_status(self, msg):
        self.status_label.config(text=msg.data)

    # Función para actualizar la imagen de la cámara
    def update_camera_image(self, msg):
        try:
            # Convertir la imagen de ROS a OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convertir la imagen de BGR a RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convertir la imagen de OpenCV a un objeto PIL
            pil_image = PILImage.fromarray(rgb_image)
            pil_image = pil_image.resize((640,480),PILImage.LANCZOS)

            # Convertir la imagen de PIL a formato compatible con Tkinter
            tk_image = ImageTk.PhotoImage(pil_image)

            # Actualizar la imagen en la interfaz
            self.camera_label.config(image=tk_image)
            self.camera_label.image = tk_image  # Mantener una referencia para evitar que se recoja la basura

        except Exception as e:
            rospy.logerr("Error al procesar la imagen: %s", str(e))

    # Funciones para los comandos del robot
    def send_coordinates(self):
        # Obtener las coordenadas desde el campo de texto o desde la habitación seleccionada
        room = self.room_selector.get()
        if room != "Seleccionar destino":
            self.pub_voice.publish(room)  # Publicar el lugar seleccionado al topic /voice_commands_persona
            rospy.loginfo(f"Comando enviado: {room}")

    def move_forward(self):
        self.send_command("adelante")

    def move_backward(self):
        self.send_command("atrás")

    def turn_left(self):
        self.send_command("izquierda")

    def turn_right(self):
        self.send_command("derecha")

    def stop_robot(self):
        self.send_command("detener")

    def resume_robot(self):
        self.send_command("reanudar")

    def send_command(self, command):
        self.pub_movement.publish(command)
        self.pub_voice.publish(command)

# Código principal para ejecutar la aplicación
if __name__ == '__main__':
    root = tk.Tk()
    app = RosTkinterApp(root)
    root.mainloop()
