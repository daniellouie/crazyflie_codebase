# opti_track_subscriber.py
import rclpy

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16

import numpy as np
from scipy.spatial.transform import Rotation as R


class OptiTrackSubscriber(Node):
    def __init__(self):
        super().__init__('opti_track_subscriber')

        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf/pose',  # Replace with your actual topic name
            self.listener_callback,
            qos_profile
        
        )
        self.pub_thrust = self.create_publisher(UInt16, '/cf1/thrust', 10)
        self.position = [0.0, 0.0, 0.0]
        self.target_position = [0.0,1.2,0.0]



        # Values for rotational (yaw) PID
        self.orientation_quat = [0.0, 0.0, 0.0, 0.0] #current orientation in quaternions
        self.current_orientation = 0.0
        self.target_orientation = -60.0
        self.k_p_rot = 0.04

        self.intErr = 0.0
        self.prevErr = 0.0

        # values for vertical (Y) PID
        self.hover = 46500      
        self.k_p = 10000
        self.k_i = 5000
        self.k_d = 10000

    def listener_callback(self, msg):
        # need this conditional to avoid QoS error
        if msg.header.frame_id == "world":
            self.position[0] = msg.pose.position.x
            self.position[1] = msg.pose.position.y
            self.position[2] = msg.pose.position.z

            self.orientation_quat[0] = msg.pose.orientation.x
            self.orientation_quat[1] = msg.pose.orientation.y
            self.orientation_quat[2] = msg.pose.orientation.z
            self.orientation_quat[3] = msg.pose.orientation.w

            r = R.from_quat(self.orientation_quat)
            pitch, yaw, roll = r.as_euler('xyz', degrees = True)
            self.current_orientation = yaw
            rot_error = self.target_orientation - self.current_orientation
            #print(f"rot_error: {rot_error}")

            yawrate = self.k_p_rot * rot_error
            #print(f"yawrate initial: {yawrate}")

            yawrate = max(-15, min(yawrate, 15))
            print(f"yawrate in range: {yawrate}")

            y_error = self.target_position[1] - self.position[1]
            # self.calculate_thrust(y_error)
            #print(f"y_error: {y_error}")

            # k_p = 10000  # Proportional gain (change this back to ~10000 after test)

            # int_error = int_error + 0.5 * (self.prevErr + error) * t
            # der_error = (error-self.prevErr) / t
            # thrust = 46500 + self.k_p * error + self.k_i * int_error + self.k_d * der_error
            # self.prevErr = error
            thrust = self.hover + self.k_p * y_error

            # Clamp thrust to valid values (37000 is a rough hover thrust value)
            thrust = int(max(42000, min(thrust, 50000)))

            msg = UInt16()
            msg.data = thrust
            self.pub_thrust.publish(msg)

        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass
    
    # def calculate_thrust(self, y_error):
    #     # Simple proportional controller
    #     k_p = 10000  # Proportional gain
    #     error = y_error
    #     thrust = 35000 + k_p * error
        
    #     # Clamp thrust to valid values (37000 is a rough hover thrust value)
    #     thrust = max(30000, min(thrust, 40000))

    #     msg = UInt16
    #     msg.data = thrust
    #     self.pub_thrust.publish(msg)
   

    def get_position(self):
        return self.position
    
def main(args=None):
    rclpy.init(args=args)

    optitrack_subscriber = OptiTrackSubscriber()

    rclpy.spin(optitrack_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optitrack_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  
