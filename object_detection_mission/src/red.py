#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import numpy as np

# Cargar la red neuronal entrenada (MobileNet SSD)
PROTOTXT_PATH = "/home/hp/Proyecto_moviles/src/object_detection_mission/MobileNetSSD_deploy.prototxt"
MODEL_PATH = "/home/hp/Proyecto_moviles/src/object_detection_mission/MobileNetSSD_deploy.caffemodel"
net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATH, MODEL_PATH)

# Lista de clases relevantes para este caso
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

# Mapeo de clases para incluir "bin"
CUSTOM_CLASSES = CLASSES.copy()
CUSTOM_CLASSES[16] = "bin"  # Suponiendo que "pottedplant" es la clase 16

class TurtleBot2ObjectDetectionNode:
    def __init__(self, confidence_threshold=0.5):
        rospy.init_node('turtlebot2_object_detection')

        self.bridge = CvBridge()
        self.detected_objects = []
        self.confidence_threshold = confidence_threshold

        # Suscriptor para Kinect del TurtleBot 2
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Publicador para objetos detectados
        self.object_pub = rospy.Publisher('/detected_objects', PoseStamped, queue_size=10)
        self.shape_pub = rospy.Publisher('/detected_shapes', String, queue_size=10)

        # Buffer y listener de transformaciones TF para TurtleBot 2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Nodo de detección de objetos para TurtleBot 2 iniciado.")

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
                if idx >= len(CUSTOM_CLASSES):
                    rospy.logwarn(f"Class ID {idx} fuera de rango para CUSTOM_CLASSES con tamaño {len(CUSTOM_CLASSES)}")
                    continue

                box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                (startX, startY, endX, endY) = box.astype("int")

                results.append((idx, confidence, (startX, startY, endX, endY)))

        return results

    def image_callback(self, msg):
        """
        Callback para procesar la imagen de la cámara.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")
            return

        detections = self.detect_objects(cv_image)
        for class_id, confidence, box in detections:
            startX, startY, endX, endY = box
            label = CUSTOM_CLASSES[class_id]
            rospy.loginfo(f"Detección: {label} con confianza {confidence:.2f} en posición x={startX}, y={startY}, w={endX-startX}, h={endY-startY}")

            # Calcular la posición del objeto en el marco de referencia de la cámara
            object_position_camera = PointStamped()
            object_position_camera.header.frame_id = "camera_link"
            object_position_camera.header.stamp = rospy.Time.now()
            object_position_camera.point.x = (startX + endX) / 2
            object_position_camera.point.y = (startY + endY) / 2
            object_position_camera.point.z = 0

            try:
                # Transformar la posición del objeto al marco de referencia del mapa
                object_position_map = self.tf_buffer.transform(object_position_camera, "map", rospy.Duration(1.0))

                # Verificar si el objeto ya ha sido detectado
                duplicate = False
                for obj in self.detected_objects:
                    distance = np.sqrt((obj.pose.position.x - object_position_map.point.x)**2 +
                                       (obj.pose.position.y - object_position_map.point.y)**2)
                    if distance < 0.5:  # Umbral de distancia para considerar duplicados
                        duplicate = True
                        break

                if not duplicate:
                    # Publicar la detección
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = object_position_map.point.x
                    pose.pose.position.y = object_position_map.point.y
                    pose.pose.position.z = object_position_map.point.z

                    # Convertir el array de quaternion a un objeto Quaternion
                    quaternion = quaternion_from_euler(0, 0, 0)
                    pose.pose.orientation = Quaternion(*quaternion)
                    self.object_pub.publish(pose)

                    shape_msg = String()
                    shape_msg.data = label
                    self.shape_pub.publish(shape_msg)

                    # Añadir el objeto detectado a la lista
                    self.detected_objects.append(pose)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Error al transformar la posición del objeto: {e}")

if __name__ == '__main__':
    try:
        confidence_threshold = 0.8  # Ajusta este valor según sea necesario
        node = TurtleBot2ObjectDetectionNode(confidence_threshold=confidence_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
           
