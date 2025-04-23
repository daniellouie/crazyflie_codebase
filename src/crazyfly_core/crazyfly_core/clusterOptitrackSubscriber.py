import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class ClusterOptitrackSubscriber(Node):
    def __init__(self, drone_id):
        """
        Initialize the OptiTrackSubscriber for a specific drone.

        :param drone_id: ID of the drone (e.g., 'cf1', 'cf2').
        """
        super().__init__(f'cluster_optitrack_subscriber_{drone_id}')


        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.drone_id = drone_id
        self.position = [0.0, 0.0, 0.0]  # Current position of the drone
        self.orientation = [0.0, 0.0, 0.0, 0.0]  # Current orientation (quaternion)

        # Subscribe to the drone's pose topic
        topic_name = f'/vrpn_mocap/{drone_id}/pose'
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        """
        Callback function to update the position and orientation of the drone.
        """
        if msg.header.frame_id == "world":
            # Update position
            self.position[0] = msg.pose.position.x
            self.position[1] = msg.pose.position.y
            self.position[2] = msg.pose.position.z
            # Update orientation (quaternion)
            self.orientation[0] = msg.pose.orientation.x
            self.orientation[1] = msg.pose.orientation.y
            self.orientation[2] = msg.pose.orientation.z
            self.orientation[3] = msg.pose.orientation.w

    def get_position(self):
        """
        Get the current position of the drone.
        :return: List of [x, y, z] position.
        """
        return self.position.copy()


    def get_orientation(self):
        """
        Get the current orientation of the drone.
        :return: List of [x, y, z, w] quaternion orientation.
        """
        return self.orientation.copy()
