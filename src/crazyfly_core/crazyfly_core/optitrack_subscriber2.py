# optitrack_subscriber2.py for second drone 

''' This file tunes the individual drone, cf2'''


## Last Update: Sept 9, 2025

## What was changed:
# Gains for CF2 got updated
# Logic for multiple waypoints was updated

## Changes left to make:
# clean up tuning on cf2

# big comments blobs were deleted, need to continue renaming variables and cleaning up code

# 0)  IMPORT RELEVANT LIBRARIES

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import csv
from rclpy.logging import get_logger
from datetime import datetime
from .flightplots import FILE_INITIATION
import math
from math import atan2, degrees
import numpy as np
from scipy.spatial.transform import Rotation as R


# 1) Pointing CF2 towards relevant path


CF2_PID =  os.path.expanduser("~/crazyfly_ws/cf_tuning_data/cf1_new_config_pid_values") 


# 2) Creates the class of OptiTrackSubsciber

''' This publishes to cf2_tuning_flight_data '''


class OptiTrackSubscriber2(Node):
    def __init__(self):
        super().__init__('opti_track_subscriber2')

        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf1/pose',
            self.listener_callback,
            qos_profile
        
        )

        # message to send flight commands to crazyflie program
        self.pub_commands = self.create_publisher(Float32MultiArray, '/cf1/commands', 10)
        self.pub_controller_pid_details = self.create_publisher(Float32MultiArray, 'cf1/controller_pid_details', 10)


        # drone to drone communication for waypoint synchronization
        self.pub_threshold_met = self.create_publisher(Bool, '/threshold_met_cf2', 10)
        self.sub_threshold_met = self.create_subscription(Bool, '/threshold_met_cf1', self.cf1_threshold_met_callback, 10)  
        self.threshold_met = False
        self.cf1_threshold_met = False

        #INITIAL SET UP 
        self.position = [0.0, 0.0, 0.0] #current position of drone, automatically updated

        self.target_positions = [[1.0, 1.0, 1.0]] #set single position (x,y,z)
        # self.target_positions = [[1.0, 1.0, 2.0], [1.0, 1.0, 3.0]] 
        self.target_pitch_deg = 0.0
        self.target_roll_deg = 0.0
        
        # Controls variables
        self.current_target_index = 0 
        self.target_position = self.target_positions[self.current_target_index]
        self.threshold = 0.30  # [m] Threshold for reaching the target
        
        # Changing self.t so that I and D, when going on delta T, go on accurate delta T and not a fixed constant
        self.t = 0.01 #average time between signals in seconds
        self.last_cb_time = self.get_clock().now()
        self.dt = 0.1

        # Values for rotational (yaw) PID
        self.orientation_quat = [0.0, 0.0, 0.0, 1.0] #current orientation in quaternions
        self.current_orientation = 0.0
        self.target_orientation_quat = [0.0, 0.0, 0.0, 1.0]

        self.target_orientation = 0.0
        self.k_p_rot = 1.0
        self.k_p_rot_sign = 1

        # CF2 GAINS

        # ───────────────────────────  PID GAIN & CONSTANTS BLOCK  ────────────────────────────

        # temp feedforward
        # self.pitch_feedforward = -0.7 # -0.5
        # self.roll_feedforward = -0.4 # -1.2 # -1.9
        
        self.pitch_feedforward = 0.0 # -0.5
        self.roll_feedforward = 0.0 # -1.2 # -1.9

        # --
                # ----------------------         Y constants         ---------------------- #
        # values for vertical Y (thrust) PID  ── ALTITUDE LOOP (tuned 2025-07-14)
      
        
        self.hover       = 41500      # trim thrust to hold level hover
        self.max_thrust  = 44000
        self.min_thrust  = 36000
        
        
        # self.hover       = 39000      # trim thrust to hold level hover
        # self.max_thrust  = 45000
        # self.min_thrust  = 36000

        self.k_p_y       = 22000     #7450 
        self.k_i_y       = 1000     #0    
        self.k_d_y       = 16000     #9700 

        self.max_yawrate = 15
        self.min_yawrate = -15

        # NEVER CHANGES
        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.int_y_max = 5000 # maximum added thrust from integral component
        

                # ----------------------         X constants         ---------------------- #
       
        # self.k_p_x       = 1.0    
        # self.k_i_x       = 0.15
        # self.k_d_x       = 3.2 # 3.25, 3.75
          
        self.k_p_x       = 4.5
        self.k_i_x       = 0.0
        self.k_d_x       = 4.0


        self.max_pitch   = 4.0
        self.min_pitch   = -4.0

        # NEVER CHANGES
        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.int_x_max = 3.0 # maximum added pitch from integral component
        # --

       
                # ----------------------         Z constants         ---------------------- #
       
        #NOTE: CHANGING VALUES FOR NEW CF2 MARKER
        self.k_p_z       = -4.5
        self.k_i_z       = -0.0
        self.k_d_z       = -4.0

        self.max_roll = 3.0
        self.min_roll = -3.0 

        # NEVER CHANGES
        self.cur_z_error = 0.0
        self.prev_z_error = 0.0
        self.int_z_error = 0.0
        self.int_z_max = 4.0
        # --
        # ────────────────────────────────────────────────────────────────────────

        self.startTimer = False
        self.startTime = time.time()


    def normalize_quat(self, q):
            x,y,z,w = q
            norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
            if norm < 1e-9:
                return [0.0, 0.0, 0.0, 1.0]
            return [x/norm,y/norm,z/norm, w/norm]
        
    def set_target_orientation_quat(self, q):
        self.target_orientation_quat = self.normaliz_quat(q)

    # Saves all PID gain values to csv in another directory
    def save_pid(self):
        time_s = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") #creates timestamp for every file
        cf2_path = FILE_INITIATION("cf2_path")
        cf2_tuning_name = os.path.basename(cf2_path)
        #fname = f"cf2_pid_{cf2_tuning_name[11:30]}.csv" #name of csv
        fname = f"cf2_pid_{time_s}.csv"
        path = os.path.join(CF2_PID, fname)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
        # logger = get_logger("cf_pid_logger")
        # logger.info(f"---------------------------------PID WRITE TO {path}")
        with open(path, "w", newline="") as file:
            w = csv.writer(file)
            w.writerow([
                "time", "hover", "max_thrust", "min_thrust",
                "k_p_x", "k_i_x", "k_d_x",
                "k_p_y", "k_i_y", "k_d_y",
                "k_p_z", "k_i_z", "k_d_z"
            ])

            w.writerow([
                time_s,
                self.hover, self.max_thrust, self.min_thrust,
                self.k_p_x, self.k_i_x, self.k_d_x,
                self.k_p_y, self.k_i_y, self.k_d_y,
                self.k_p_z, self.k_i_z, self.k_d_z
            ])     

    def listener_callback(self, msg):

        # # Overwriting self.t to be real frequency time for ID terms
        # now = self.get_clock().now()
        # dt = (now - self.last_cb_time).nanoseconds * 1e-9  # seconds
        # self.last_cb_time = now

        # # Clamp dt to keep the controller sane on the first sample or hiccups
        # if not (1e-4 <= dt <= 0.1):          # expected ~100 Hz -> 0.01 s
        #     dt = self.dt                     # fall back to last dt if crazy
        # self.dt = dt
        # self.t = dt

        # need this conditional to avoid QoS error
        if msg.header.frame_id == "world":
            # store the current x,y,x position of the drone (in meters)
            self.position[0] = msg.pose.position.x
            self.position[1] = msg.pose.position.y
            self.position[2] = msg.pose.position.z

            # store the current x, y, z, w orientation of the drone (in Quaternions)
            self.orientation_quat[0] = msg.pose.orientation.x
            self.orientation_quat[1] = msg.pose.orientation.y
            self.orientation_quat[2] = msg.pose.orientation.z
            self.orientation_quat[3] = msg.pose.orientation.w

            # Grab world orientation quaternion (for relative drone orientation)
            q_world = R.from_quat(self.orientation_quat)

            # calls rotational PID function (yawrate)
            yawrate_cmd = self.calculate_yawrate()
            # self.get_logger().info(f"yawrate_cmd = {yawrate_cmd:.2f}")

            # calls X axis PID function (pitch)
            pitch_cmd, pitch_pid_details = self.calculate_pitch()

            # calls Y axis PID function (thrust)
            thrust, thrust_pid_details = self.calculate_thrust()

            # Calls Z axis PID function (roll)
            roll_cmd, roll_pid_details = self.calculate_roll()

            # pitch_x = optitrack frame roll
            # yaw_y = optitrack frame pitch
            # roll_z = opitrack frame yaw
            r = R.from_quat(self.orientation_quat)
            pitch_x, yaw_y, roll_z = r.as_euler('xyz', degrees = True)

            # create message of type Float Array (all values need to be floats)

            msg = Float32MultiArray()
            msg.data = [float(roll_cmd), float(pitch_cmd), float(yawrate_cmd), float(thrust),      # why is this roll pitch yaw??
                        float(self.position[0]), float(self.position[1]), float(self.position[2]),
                        float(yaw_y), float(pitch_x), float(roll_z),
                        float(self.target_position[0]-self.position[0]), float(self.target_position[1]-self.position[1]),
                        float(self.target_position[2]-self.position[2])]

            # msg.data = [float(roll_cmd), float(pitch_cmd), float(yawrate_cmd), float(thrust),      # why is this roll pitch yaw??
            #             float(self.position[0]), float(self.position[1]), float(self.position[2]),
            #             float(yaw_y), float(pitch_x), float(roll_z)]
            
            # print(msg.data)
            self.pub_commands.publish(msg) #publish commands for drone controller

            msg = Float32MultiArray()
            msg.data = [float(pitch_pid_details[0]), float(pitch_pid_details[1]), float(pitch_pid_details[2]),
                        float(roll_pid_details[0]), float(roll_pid_details[1]), float(roll_pid_details[2]),
                        float(thrust_pid_details[0]), float(thrust_pid_details[1]), float(thrust_pid_details[2])]
            self.pub_controller_pid_details.publish(msg)

            #new threshold logic
            if self.is_within_threshold(self.position, self.target_position): #if drone is at desired position
                if not self.startTimer: #if the timer for hovering has not started, start it
                    print("cf2 timer started")
                    self.startTimer = True
                    self.startTime = time.time()
                elif time.time() - self.startTime >= 4: #if the drone has been at the desired position for 3 seconds
                    if not self.threshold_met:
                        self.threshold_met = True
                        self.publish_threshold_met()
                        print("cf2: Threshold met")
                    if self.threshold_met:
                        self.current_target_index += 1
                        if self.current_target_index < len(self.target_positions):  #if these is another target position, move to it
                            self.target_position = self.target_positions[self.current_target_index]
                            self.get_logger().info(f"cf2:Moving to next target position {self.target_position}") 
                            print(f"cf2: moving to next position: {self.target_position}")
                    else:
                        print("Waiting for cf1 to reach threshold.")

        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass

    # helps drone move from one position to the other 
    def is_within_threshold(self, position, target_position):
        # Calculate Euclidean distance between the current position and target position
        dist = np.linalg.norm(np.array(position) - np.array(target_position))
        return dist <= self.threshold
    
    # callback function that gets the threshold_met value from cf1
    def cf1_threshold_met_callback(self, msg):
        self.cf1_threshold_met = msg.data #set the boolean to the value recieved from cf2

    # publishes the threshold_met value to communicate with cf1
    def publish_threshold_met(self):
        threshold_met_msg = Bool()
        threshold_met_msg.data = self.threshold_met
        self.pub_threshold_met.publish(threshold_met_msg)

    # rotational control
    def calculate_yawrate(self):
        r = R.from_quat(self.orientation_quat)
        pitch_x, yaw_y, roll_z = r.as_euler('xyz', degrees = True)
        self.pitch_meas = pitch_x
        self.yaw_meas = yaw_y
        self.roll_meas = roll_z
        self.current_orientation = self.yaw_meas
        rot_error = (self.target_orientation - self.yaw_meas + 180) % 360 - 180    # NOTE: logic is changed here 8/4/25
        yawrate_cmd = np.clip(self.k_p_rot * rot_error, self.min_yawrate, self.max_yawrate)

        # workaround logic to determine direction of rotation (bc of quaternions)
        if abs(self.orientation_quat[1]) > 0.7: # this value is specific to a certain set up orientation
            self.k_p_rot_sign = 1
        else:
            self.k_p_rot_sign = -1
        yawrate_cmd = self.k_p_rot_sign * self.k_p_rot * rot_error
        yawrate_cmd = max(self.min_yawrate, min(yawrate_cmd, self.max_yawrate))

        return yawrate_cmd
    
    # FIXED Z axis control 8/19/25
    def calculate_pitch(self):
    
        # set to zero if within margin
        self.cur_z_error = self.target_position[2] - self.position[2]
        # if -0.01 <= self.cur_z_error <= 0.01:
        #     self.cur_z_error = 0 

        # (P term)
        z_fp = self.k_p_z * self.cur_z_error # (deg/m) 

        # (I term))
        future_int_z_error = self.int_z_error + 0.5 * (self.prev_z_error + self.cur_z_error) * self.t #units of m*s 
        if abs(future_int_z_error * self.k_i_z) < self.int_z_max:
            self.int_z_error = future_int_z_error 
        z_fi = self.k_i_z * self.int_z_error # (to be in degrees Ki_z must be in deg/m*s))
        
        # (D term)
        z_error_dif = self.cur_z_error - self.prev_z_error #m
        z_fd = self.k_d_z * (z_error_dif) / self.t # gain * m/s -> gain= deg/m/s or deg*s/m
        self.prev_z_error = self.cur_z_error # (deg*s/m)
        
        desired_pitch_angle = self.pitch_feedforward + z_fp + z_fi + z_fd # desired pitch angle in DEGREESSSS
 
        # return pitch_cmd
        return desired_pitch_angle, [z_fp, z_fi, z_fd]

    # Y axis control 
    def calculate_thrust(self):
        # P term:
        self.cur_y_error = self.target_position[1]- self.position[1]

        # I term:
        # if -0.01 <= self.cur_y_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
        #     self.cur_y_error = 0

        y_fp = self.k_p_y * self.cur_y_error

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
        return thrust, [y_fp, y_fi, y_fd]
    
    # FIXED X-axis control. 08/19/25
    def calculate_roll(self):

        # set to zero if within margin
        self.cur_x_error = self.target_position[0] - self.position[0]
        # if -0.01 <= self.cur_x_error <= 0.01:
        #     self.cur_x_error = 0

        # (P term)
        x_fp = self.k_p_x * self.cur_x_error # (deg/m) 

        # (I term))
        future_int_x_error = self.int_x_error + 0.5 * (self.prev_x_error + self.cur_x_error) * self.t #units of m*s
        # print(f"future_int_x_error: {future_int_x_error}")
        if abs(future_int_x_error * self.k_i_x) < self.int_x_max:
            self.int_x_error = future_int_x_error 
        x_fi = self.k_i_x * self.int_x_error # (to be in degrees Ki_x must be in deg/m*s))
        
        # (D term)
        x_error_dif = self.cur_x_error - self.prev_x_error #m
        x_fd = self.k_d_x * (x_error_dif) / self.t # gain * m/s -> gain= deg/m/s or deg*s/m
        self.prev_x_error = self.cur_x_error # (deg*s/m)
        
        desired_roll_angle = self.roll_feedforward + x_fp + x_fi + x_fd # desired roll angle in DEGREESSSS
        
        # return roll_cmd

        return desired_roll_angle, [x_fp, x_fi, x_fd]

    def get_position(self):
        return self.position
    
def main(args=None):
    rclpy.init(args=args)
    optitrack_subscriber2 = OptiTrackSubscriber2()

    try:
        optitrack_subscriber2.save_pid()
        rclpy.spin(optitrack_subscriber2)
    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt")
    finally:
        optitrack_subscriber2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  