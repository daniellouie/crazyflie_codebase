# opti_track_subscriber.py
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
import time

import numpy as np
from scipy.spatial.transform import Rotation as R


class PIDControllerCF2(Node):
    def __init__(self):
        super().__init__('pid_controller_cf2')

        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf2/pose',
            self.current_position_callback,
            qos_profile
        
        )
        self.create_subscription(
            PoseStamped,
            '/cf2_cmd',  # Commanded position from clusterController
            self.commanded_position_callback,
            10
        )

        #message to send flight commands to crazyflie program
        self.pub_commands = self.create_publisher(
            Float32MultiArray, 
            '/cf2/commands', 
            10
        )

        #INITIAL SET UP (just using current and target positions for now)
        self.current_position = [0.0, 0.0, 0.0] #current position of drone, automatically updated
        self.commanded_position = [0.0, 0.0, 0.0] # commanded position from the cluster controller
        self.constant_position = [1.0, 1.0, 1.0] # constant position for testing
  
        # Controls variables
        self.t = 0.01 #average time between signals in seconds
        
        # Values for rotational (yaw) PID
        self.orientation_quat = [0.0, 0.0, 0.0, 0.0] #current orientation in quaternions
        self.current_orientation = 0.0
        self.target_orientation_quat = [0.0, 0.7, 0.0, 0.7]
        #temp fix: need to rotate drone to face right for rigid body then reorient
        self.target_orientation = 90 #position the drone in desired orientation and this value should be the yaw from optitrack
        self.k_p_rot = 0.25
        self.k_p_rot_sign = 1
        self.max_yawrate = 15
        self.min_yawrate = -15

        # values for vertical Y (thrust) PID
        self.hover = 44000 #originally 46500     
        self.max_thrust = 56000 #origionall 50000
        self.min_thrust = 42000
        self.k_p_y = 30000 #previously 19000 on May 6 
        self.k_i_y = 1500 #extra amount of thrust wanted (originally 2000)
        self.k_d_y = 10500 #previously 10000
        self.threshold_met = False

        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.int_y_max = 5000 # maximum added thrust from integral component

        # values for horizontal X (pitch) PID
        self.k_p_x = 2
        self.k_i_x = 0.6
        self.k_d_x = 4.0 #previously 4.0 May 6 
        self.max_pitch = 3.0
        self.min_pitch = -3.0

        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.int_x_max = 3.0 # maximum added pitch from integral component

        # values for horizontal Z (roll) PID
        self.k_p_z = 2.5 #Origionally 2 May 6 
        self.k_i_z = 0.6
        self.k_d_z = 4.1 #previously 4.0 May 6 
        self.max_roll = 3.0
        self.min_roll = -3.0

        self.cur_z_error = 0.0
        self.prev_z_error = 0.0
        self.int_z_error = 0.0
        self.int_z_max = 3.0


        self.startTimer = False
        self.startTime = time.time()

    def current_position_callback(self, msg):
        # print(f"Received pose: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        # need this conditional to avoid QoS error
        if msg.header.frame_id == "world":
            # store the current x,y,x position of the drone (in meters)
            self.current_position[0] = msg.pose.position.x
            self.current_position[1] = msg.pose.position.y
            self.current_position[2] = msg.pose.position.z

            # store the current x, y, z, w orientation of the drone (in Quaternions)
            self.orientation_quat[0] = msg.pose.orientation.x
            self.orientation_quat[1] = msg.pose.orientation.y
            self.orientation_quat[2] = msg.pose.orientation.z
            self.orientation_quat[3] = msg.pose.orientation.w
            # print(f"Current position callback: {self.current_position}")

            self.calculate_motor_commands() # call the function to calculate motor commands

        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass
    
    def commanded_position_callback(self, msg):
        if msg.header.frame_id == "world":
            self.commanded_position[0] = msg.pose.position.x #use actual commanded position
            # self.commanded_position[0] = self.constant_position[0] # use constant value for testing
            self.commanded_position[1] = msg.pose.position.y  #use actual commanded position
            # self.commanded_position[1] = self.constant_position[1] # use constant value for testing
            self.commanded_position[2] = msg.pose.position.z  #use actual commanded position
            # self.commanded_position[2] = self.constant_position[2] # use constant value for testing

            # print(f"Commanded position callback: {self.commanded_position}")
        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass

    # function to call calculation functions and publish the commands
    def calculate_motor_commands(self):
        # calls rotational PID function (yawrate)
        yawrate = self.calculate_yawrate()

        # calls X axis PID function (pitch)
        pitch = self.calculate_pitch()

        # calls Y axis PID function (thrust)
        thrust = self.calculate_thrust()

        # Calls Z axis PID function (roll)
        roll = self.calculate_roll()

        # create message of type Float Array (all values need to be floats)
        msg = Float32MultiArray()
        msg.data = [float(roll), float(pitch), float(yawrate), float(thrust), float(self.current_position[0]), float(self.current_position[1]), float(self.current_position[2])]
        # print(f"roll: {roll}, \n pitch: {pitch}, \n yawrate: {yawrate}, \n thrust: {thrust}")
        self.pub_commands.publish(msg) #publish commands for drone controller
            
    

    # rotational control
    def calculate_yawrate(self):
        # (P term)
        r = R.from_quat(self.orientation_quat)
        pitch, yaw, roll = r.as_euler('xyz', degrees = True)
        self.current_orientation = yaw
        rot_error = self.target_orientation - self.current_orientation

        # workaround logic to determine direction of rotation (bc of quaternions)
        if abs(self.orientation_quat[1]) > 0.7: # this value is specific to a certain set up orientation
            self.k_p_rot_sign = 1
        else:
            self.k_p_rot_sign = -1

        yawrate = self.k_p_rot_sign * self.k_p_rot * rot_error
        yawrate = max(self.min_yawrate, min(yawrate, self.max_yawrate))
        return yawrate
    
    # X Axis control
    def calculate_pitch(self):
        # (P term)
        self.cur_x_error = self.commanded_position[0] - self.current_position[0]
        if -0.01 <= self.cur_x_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
            self.cur_x_error = 0
        x_fp = self.k_p_x * self.cur_x_error
        # print(f"x_fp: {x_fp}")
        
        # I term
        future_int_x_error = self.int_x_error + 0.5 * (self.prev_x_error + self.cur_x_error) * self.t
        if abs(future_int_x_error * self.k_i_x) < self.int_x_max:
            self.int_x_error = future_int_x_error
        x_fi = self.k_i_x * self.int_x_error
        # print(f"x_fi: {x_fi}")

        # D term
        x_error_dif = self.cur_x_error - self.prev_x_error
        x_fd = self.k_d_x * (x_error_dif) / self.t
        # print(f"x_fd: {x_fd}")
        self.prev_x_error = self.cur_x_error

        pitch = x_fp + x_fi + x_fd
        pitch = max(self.min_pitch, min(pitch, self.max_pitch))
        return pitch
    
    # Y axis control 
    def calculate_thrust(self):
        # P term:
        self.cur_y_error = self.commanded_position[1]- self.current_position[1]

        # I term:
        #if -0.01 <= self.cur_y_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
        #    self.cur_y_error = 0

        y_fp = self.k_p_y * self.cur_y_error
        #print(f"y_fp: {y_fp}")

        #calculate what k_i would be
        future_int_y_error = self.int_y_error + 0.5 * (self.prev_y_error + self.cur_y_error) * self.t
        #if the calculated value is within range, update int_y_error
        if abs(future_int_y_error * self.k_i_y) < self.int_y_max:
            self.int_y_error = future_int_y_error
        #otherwise keep self.int_y_error the same
        y_fi = self.k_i_y * self.int_y_error
        #print(f"y_fi: {y_fi}")

        # D term
        error_dif = self.cur_y_error - self.prev_y_error
        y_fd = self.k_d_y * (error_dif) / self.t
        #print(f"y_fd: {y_fd}")
        self.prev_y_error = self.cur_y_error

        thrust = self.hover + y_fp + y_fi + y_fd
        # Clamp thrust to valid range 
        thrust = int(max(self.min_thrust, min(thrust, self.max_thrust)))
        return thrust
    
    def calculate_roll(self):
        # P term
        self.cur_z_error = self.commanded_position[2] - self.current_position[2]
        z_fp = self.k_p_z * self.cur_z_error
        #print(f"z_fp: {z_fp}")

        # I term
        future_int_z_error = self.int_z_error + 0.5 * (self.prev_z_error + self.cur_z_error) * self.t
        if abs(future_int_z_error * self.k_i_z) < self.int_z_max:
            self.int_z_error = future_int_z_error
        z_fi = self.k_i_z * self.int_z_error
        #print(f"z_fi: {z_fi}")

        # D Term
        z_error_dif = self.cur_z_error - self.prev_z_error
        z_fd = self.k_d_z * (z_error_dif) / self.t
        #print(f"z_fd: {z_fd}")
        self.prev_z_error = self.cur_z_error

        roll = z_fp + z_fi + z_fd
        roll = max(self.min_roll, min(roll, self.max_roll))
        return roll
   
    def get_position(self):
        return self.current_position
    
def main(args=None):
    rclpy.init(args=args)

    pid_controller_cf1 = PIDControllerCF2()

    rclpy.spin(pid_controller_cf1)

    pid_controller_cf1.destroy_node()
    rclpy.shutdown()

    # try:
    #     rclpy.spin(optitrack_subscriber)
    # except KeyboardInterrupt:
    #     optitrack_subscriber.get_logger().info('Node stopped by user')
    # finally:
    #     # Destroy the node explicitly
    #     # (optional - otherwise it will be done automatically
    #     # when the garbage collector destroys the node object)
    #     optitrack_subscriber.destroy_node()
    #     rclpy.shutdown()

      

if __name__ == '__main__':
    main()  
