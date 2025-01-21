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
import onnxruntime as ort

# Cargar la red neuronal entrenada (YOLOv5 pre-entrenado en COCO dataset)
MODEL_PATH = "/home/franciscocj/Proyecto_moviles/src/object_detection_mission/Yolo/yolov5s.onnx"
ort_session = ort.InferenceSession(MODEL_PATH)

# Lista de clases relevantes para este caso
CLASSES = ["circle", "square", "triangle", "bottle", "chair", "ball", "bin"]

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
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(640, 640), swapRB=True, crop=False)
        ort_inputs = {ort_session.get_inputs()[0].name: blob}
        outputs = ort_session.run(None, ort_inputs)

        # Procesar las salidas de la red neuronal
        detections = []
        for detection in outputs[0]:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                if class_id >= len(CLASSES):
                    rospy.logwarn(f"Class ID {class_id} fuera de rango para CLASSES con tamaño {len(CLASSES)}")
                    continue

                box = detection[0:4] * np.array([width, height, width, height])
                (centerX, centerY, w, h) = box.astype("int")

                x = int(centerX - (w / 2))
                y = int(centerY - (h / 2))

                detections.append((class_id, confidence, (x, y, int(w), int(h))))

        return detections

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
            x, y, w, h = box
            label = CLASSES[class_id]
            rospy.loginfo(f"Detección: {label} con confianza {confidence:.2f} en posición x={x}, y={y}, w={w}, h={h}")

            # Publicar la detección
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "camera_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
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