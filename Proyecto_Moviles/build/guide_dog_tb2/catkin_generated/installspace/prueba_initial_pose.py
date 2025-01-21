#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

# Inicializa el nodo y el publicador
rospy.init_node('initial_pose_publisher', anonymous=True)
initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

# DEPENDENCIA:
x_initial = 0  # Posición inicial en x
y_initial = 0  # Posición inicial en y
yaw_initial = 0.0  # Orientación inicial en yaw (radianes)

# Crear el mensaje PoseWithCovarianceStamped para /initialpose
initial_pose = PoseWithCovarianceStamped()
initial_pose.header.frame_id = "map"
initial_pose.header.stamp = rospy.Time.now()

# Asignar la posición
initial_pose.pose.pose.position.x = x_initial
initial_pose.pose.pose.position.y = y_initial
initial_pose.pose.pose.position.z = 0.0

# Convertir la orientación yaw a cuaternión
quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_initial)
initial_pose.pose.pose.orientation.x = quaternion[0]
initial_pose.pose.pose.orientation.y = quaternion[1]
initial_pose.pose.pose.orientation.z = quaternion[2]
initial_pose.pose.pose.orientation.w = quaternion[3]

# Configurar la covarianza (incertidumbre) en la posición inicial
initial_pose.pose.covariance[0] = 0.25  # Incertidumbre en x
initial_pose.pose.covariance[7] = 0.25  # Incertidumbre en y
initial_pose.pose.covariance[35] = 0.068537  # Incertidumbre en yaw

# Publicar la pose inicial varias veces para asegurar que AMCL la reciba
rate = rospy.Rate(1)
for i in range(10):
    initial_pose_publisher.publish(initial_pose)
    rate.sleep()

print("Initial pose sent!")
