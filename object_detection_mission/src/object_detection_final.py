#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import numpy as np

# Cargar la red neuronal entrenada (MobileNet SSD)
PROTOTXT_PATH = "/home/franciscocj/Proyecto_moviles/src/object_detection_mission/MobileNetSSD_deploy.prototxt"
MODEL_PATH = "/home/franciscocj/Proyecto_moviles/src/object_detection_mission/MobileNetSSD_deploy.caffemodel"
net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATH, MODEL_PATH)

# Lista de clases del modelo
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

# Clase para representar un objeto detectado
class DetectedObject:
    def __init__(self, x, y, object_type, confidence):
        self.x = x
        self.y = y
        self.object_type = object_type  # Tipo de objeto, como 'botella', 'silla', etc.
        self.confidence = confidence

    def __repr__(self):
        return f"Objeto({self.object_type}, x={self.x}, y={self.y}, conf={self.confidence:.2f})"

    # Función para verificar si dos objetos son similares (en función de la clase y proximidad)
    def is_similar(self, other):
        # Compara la clase y si están lo suficientemente cerca en las coordenadas
        return (self.object_type == other.object_type)


class TurtleBot2ObjectDetectionNode:
    def __init__(self, confidence_threshold=0.5):
        rospy.init_node('turtlebot2_object_detection')

        self.bridge = CvBridge()
        self.detected_objects_map = {}  # Diccionario para almacenar objetos por clase
        self.confidence_threshold = confidence_threshold

        # Suscriptor para la cámara del TurtleBot 2
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        # Publicadores
        self.object_pub = rospy.Publisher('/detected_objects', PoseStamped, queue_size=10)
        self.shape_pub = rospy.Publisher('/detected_shapes', String, queue_size=10)

        # Buffer y listener de transformaciones TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Nodo de detección de objetos para TurtleBot 2 iniciado.")

        # Variables para almacenar la imagen de profundidad
        self.depth_image = None

    def depth_callback(self, msg):
        try:
            # Convertir la imagen de profundidad en una matriz de valores flotantes (en metros)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen de profundidad: {e}")

    def detect_objects(self, frame):
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        results = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.confidence_threshold:
                idx = int(detections[0, 0, i, 1])
                if idx >= len(CLASSES):
                    rospy.logwarn(f"Class ID {idx} fuera de rango para CLASSES con tamaño {len(CLASSES)}")
                    continue

                box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                (startX, startY, endX, endY) = box.astype("int")

                results.append((idx, confidence, (startX, startY, endX, endY)))

        return results

    def get_depth_at_pixel(self, x, y):
        """Obtener la profundidad en el píxel (x, y) de la imagen de profundidad."""
        if self.depth_image is None:
            return None
        # Retornar el valor de profundidad en el píxel (x, y)
        depth_value = self.depth_image[y, x]
        return depth_value

    def calculate_3d_position(self, u, v, depth):
        """Calcular la posición 3D en el marco de referencia de la cámara."""
        # Parámetros intrínsecos de la cámara (ajustar según tu cámara)
        fx = 525.0  # Focal en X (ajusta según tu cámara)
        fy = 525.0  # Focal en Y (ajusta según tu cámara)
        cx = 319.5  # Centro de la imagen en X (ajusta según tu cámara)
        cy = 239.5  # Centro de la imagen en Y (ajusta según tu cámara)

        # Calcular las coordenadas 3D
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)

    def transform_to_global(self, local_position):
        try:
            local_pose = PoseStamped()
            local_pose.header.frame_id = 'camera_rgb_optical_frame'
            local_pose.header.stamp = rospy.Time.now()
            local_pose.pose.position.x = local_position[0]
            local_pose.pose.position.y = local_position[1]
            local_pose.pose.position.z = local_position[2]

            # Transformación a marco global 'map'
            transform = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            global_pose = tf2_geometry_msgs.do_transform_pose(local_pose, transform)

            return (global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z)
        except Exception as e:
            rospy.logwarn(f"Error en transformación: {e}")
            return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")
            return

        detections = self.detect_objects(cv_image)
        for class_id, confidence, box in detections:
            startX, startY, endX, endY = box
            label = CLASSES[class_id]
            rospy.loginfo(f"Detección: {label} con confianza {confidence:.2f} en posición x={startX}, y={startY}, w={endX-startX}, h={endY-startY}")

            # Obtener la profundidad en el centro del objeto detectado
            center_x = (startX + endX) // 2
            center_y = (startY + endY) // 2
            depth_value = self.get_depth_at_pixel(center_x, center_y)

            if depth_value is None:
                rospy.logwarn("No se pudo obtener la profundidad en el centro del objeto.")
                continue

            # Calcular la posición 3D del objeto en el sistema de coordenadas de la cámara
            local_position = self.calculate_3d_position(center_x, center_y, depth_value)

            # Transformar la posición local a global
            global_position = self.transform_to_global(local_position)

            if global_position:
                # Verificar si ya existe un objeto de esta clase con posiciones similares
                existing_object = self.detected_objects_map.get(label)
                if existing_object and existing_object.is_similar(DetectedObject(global_position[0], global_position[1], label, confidence)):
                    rospy.loginfo(f"Objeto con clase {label} ya ha sido detectado de manera similar")
                    continue

                # Publicar la detección
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = global_position[0]
                pose.pose.position.y = global_position[1]
                pose.pose.position.z = global_position[2]

                quaternion = quaternion_from_euler(0, 0, 0)
                pose.pose.orientation = Quaternion(*quaternion)
                self.object_pub.publish(pose)

                shape_msg = String()
                shape_msg.data = label
                self.shape_pub.publish(shape_msg)

                # Guardar el objeto detectado
                self.detected_objects_map[label] = DetectedObject(global_position[0], global_position[1], label, confidence)

if __name__ == '__main__':
    try:
        confidence_threshold = 0.9  # Ajusta según sea necesario
        node = TurtleBot2ObjectDetectionNode(confidence_threshold=confidence_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
