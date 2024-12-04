#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np

class ObjectDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detected_objects = []  # Lista para guardar objetos detectados

        # Suscribirse al tópico de la cámara
        rospy.Subscriber('/image/rgb/image_raw', Image, self.image_callback)   # Comprobar que este es el topic bueno

        # Publicador para objetos detectados
        self.object_pub = rospy.Publisher('/detected_objects', PoseStamped, queue_size=10)

        # Crear buffer y listener para transformaciones TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Nodo de detección de objetos iniciado.")

    def image_callback(self, msg):
        # Convertir la imagen ROS a OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")
            return

        # Detectar objetos en la imagen
        objects = self.detect_objects(cv_image)

        for obj in objects:
            # Transformar coordenadas locales a globales
            global_position = self.transform_to_global(obj['position'])
            if global_position:
                # Guardar en la lista de objetos detectados
                self.detected_objects.append({"shape": obj['shape'], "position": global_position})

                # Publicar la posición global como PoseStamped
                self.publish_object(obj['shape'], global_position)

        rospy.loginfo(f"Objetos detectados y transformados: {self.detected_objects}")

    def detect_objects(self, image):
        """
        Detectar objetos en una imagen basados en su forma.
        """
        detected_objects = []

        # Preprocesamiento
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Encontrar contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calcular momentos para encontrar el centroide
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Aproximar el contorno para determinar la forma
                approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                if len(approx) == 3:
                    shape = "triangle"
                elif len(approx) == 4:
                    shape = "square"
                else:
                    shape = "circle"

                # Guardar la forma detectada y su posición relativa
                detected_objects.append({"shape": shape, "position": (cX, cY)})

        return detected_objects

    def transform_to_global(self, local_position):
        """
        Transforma las coordenadas locales (marco de la cámara) al marco global (mapa).
        """
        try:
            # Crear un PoseStamped para la posición local
            local_pose = PoseStamped()
            local_pose.header.frame_id = 'camera_link'  # Marco local
            local_pose.header.stamp = rospy.Time.now()
            local_pose.pose.position.x = local_position[0]
            local_pose.pose.position.y = local_position[1]
            local_pose.pose.position.z = 0.0

            # Obtener la transformación de 'camera_link' a 'map'
            transform = self.tf_buffer.lookup_transform('map', 'camera_link', rospy.Time(0), rospy.Duration(1.0))

            # Aplicar la transformación
            global_pose = tf2_geometry_msgs.do_transform_pose(local_pose, transform)

            # Extraer las coordenadas transformadas
            x = global_pose.pose.position.x
            y = global_pose.pose.position.y

            return (x, y)
        except Exception as e:
            rospy.logwarn(f"No se pudo transformar la posición: {e}")
            return None

    def publish_object(self, shape, position):
        """
        Publicar la posición global del objeto como un mensaje PoseStamped.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = 0.0

        # Agregar información en los logs
        rospy.loginfo(f"Forma detectada: {shape}, posición global: {position}")
        self.object_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node('object_detection_node')
    node = ObjectDetectionNode()
    rospy.spin()
