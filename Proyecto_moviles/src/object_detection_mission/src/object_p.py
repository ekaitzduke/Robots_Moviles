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

class TurtleBot2ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('turtlebot2_object_detection')

        self.bridge = CvBridge()
        self.detected_objects = []

        # Suscriptor para Kinect del TurtleBot 2
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Publicador para objetos detectados
        self.object_pub = rospy.Publisher('/turtlebot2/detected_objects', PoseStamped, queue_size=10)

        # Buffer y listener de transformaciones TF para TurtleBot 2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Parámetros de detección ajustados para TurtleBot 2
        self.min_contour_area = 100  # Filtrar objetos muy pequeños
        self.max_contour_area = 10000  # Filtrar objetos demasiado grandes

        rospy.loginfo("Nodo de detección de objetos para TurtleBot 2 iniciado.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")
            return

        objects = self.detect_objects(cv_image)

        for obj in objects:
            global_position = self.transform_to_global(obj['position'])
            if global_position:
                self.detected_objects.append({"shape": obj['shape'], "position": global_position})
                self.publish_object(obj['shape'], global_position)

        if self.detected_objects:
            rospy.loginfo(f"Objetos detectados: {len(self.detected_objects)}")

    def detect_objects(self, image):
        detected_objects = []

        # Preprocesamiento mejorado
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 30, 200)  # Ajuste de umbrales

        # Encontrar contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filtrar por área del contorno
            area = cv2.contourArea(contour)
            if area < self.min_contour_area or area > self.max_contour_area:
                continue

            M = cv2.moments(contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Aproximación de contorno más flexible
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

                # Clasificación de formas más robusta
                if 3 <= len(approx) <= 4:
                    shape = "triangle" if len(approx) == 3 else "square"
                else:
                    # Usar elipse para detección de círculos
                    if len(approx) > 5:
                        shape = "circle"
                    else:
                        continue

                detected_objects.append({"shape": shape, "position": (cX, cY)})

        return detected_objects

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
