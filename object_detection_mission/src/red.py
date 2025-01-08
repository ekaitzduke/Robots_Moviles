#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
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

class TurtleBot2ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('turtlebot2_object_detection')

        self.bridge = CvBridge()
        self.detected_objects = []

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
            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])
                if idx >= len(CLASSES):
                    rospy.logwarn(f"Class ID {idx} fuera de rango para CLASSES con tamaño {len(CLASSES)}")
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
            label = CLASSES[class_id]
            rospy.loginfo(f"Detección: {label} con confianza {confidence:.2f} en posición x={startX}, y={startY}, w={endX-startX}, h={endY-startY}")

            # Publicar la detección
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "camera_link"
            pose.pose.position.x = (startX + endX) / 2
            pose.pose.position.y = (startY + endY) / 2
            pose.pose.position.z = 0
            pose.pose.orientation = quaternion_from_euler(0, 0, 0)
            self.object_pub.publish(pose)

            shape_msg = String()
            shape_msg.data = label
            self.shape_pub.publish(shape_msg)

if __name__ == '__main__':
    try:
        node = TurtleBot2ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
