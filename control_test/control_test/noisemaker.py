import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math
import numpy as np

class Noisemaker(Node):
    def __init__(self):
        super().__init__('noisemaker')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3']
        self.joint_state.position = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()