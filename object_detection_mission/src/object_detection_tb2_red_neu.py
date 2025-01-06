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

# Cargar la red neuronal entrenada (YOLOv5 pre-entrenado en COCO dataset)
MODEL_PATH = "yolov5s.onnx"
net = cv2.dnn.readNetFromONNX(MODEL_PATH)

# Lista de clases relevantes para este caso
CLASSES = ["circle", "square", "triangle", "bottle", "chair", "ball"]

class TurtleBot2ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('turtlebot2_object_detection')

        self.bridge = CvBridge()
        self.detected_objects = []

        # Suscriptor para Kinect del TurtleBot 2
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Publicador para objetos detectados
        self.object_pub = rospy.Publisher('/detected_objects', PoseStamped, queue_size=10)

        # Buffer y listener de transformaciones TF para TurtleBot 2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Nodo de detección de objetos para TurtleBot 2 iniciado.")

    def detect_objects(self, frame):
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(640, 640), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward()

        detections = []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:  # Umbral de confianza
                    box = detection[0:4] * np.array([width, height, width, height])
                    (center_x, center_y, box_width, box_height) = box.astype("int")
                    x = int(center_x - (box_width / 2))
                    y = int(center_y - (box_height / 2))

                    detections.append({
                        "class": CLASSES[class_id],
                        "confidence": float(confidence),
                        "box": [x, y, int(box_width), int(box_height)]
                    })
        return detections

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detections = self.detect_objects(cv_image)

            for detection in detections:
                x, y, w, h = detection["box"]
                shape = detection["class"]
                cX = x + w // 2
                cY = y + h // 2

                global_position = self.transform_to_global((cX, cY))
                if global_position:
                    self.detected_objects.append({"shape": shape, "position": global_position})
                    self.publish_object(shape, global_position)

            if self.detected_objects:
                rospy.loginfo(f"Objetos detectados: {len(self.detected_objects)}")

        except Exception as e:
            rospy.logerr(f"Error procesando la imagen: {e}")

    def transform_to_global(self, local_position):
        try:
            local_pose = PoseStamped()
            local_pose.header.frame_id = 'camera_rgb_optical_frame'  # Marco específico de TurtleBot 2
            local_pose.header.stamp = rospy.Time.now()
            local_pose.pose.position.x = local_position[0]
            local_pose.pose.position.y = local_position[1]
            local_pose.pose.position.z = 0.0

            # Transformación a marco global 'map'
            transform = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            global_pose = tf2_geometry_msgs.do_transform_pose(local_pose, transform)

            return (global_pose.pose.position.x, global_pose.pose.position.y)
        except Exception as e:
            rospy.logwarn(f"Error en transformación: {e}")
            return None

    def publish_object(self, shape, position):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = 0.0

        # Orientación por defecto
        quaternion = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Objeto {shape} en: {position}")
        self.object_pub.publish(pose)

def main():
    try:
        node = TurtleBot2ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
