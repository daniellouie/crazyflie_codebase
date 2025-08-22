# optitrack_subscriber2.py for second drone 

''' This file tunes the individual drone, cf2'''


## Last Update: Aug 22, 2025

## What was changed:
# Gains for CF2 were finalized.
# We also moved 3 files to unused (PID)

## Changes left to make:
# Delete all unused comments to improve readability



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


CF2_PID =  os.path.expanduser("~/crazyfly_ws/cf2_pid_tuning_values") 


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

        # drone to drone communication for waypoint synchronization
        self.pub_threshold_met = self.create_publisher(Bool, '/threshold_met_cf2', 10)
        self.sub_threshold_met = self.create_subscription(Bool, '/threshold_met_cf1', self.cf1_threshold_met_callback, 10)  
        self.threshold_met = False
        self.cf1_threshold_met = False

        #INITIAL SET UP 
        self.position = [0.0, 0.0, 0.0] #current position of drone, automatically updated

        self.target_positions = [1.0, 1.0, 3.0] #set single position (x,y,z)
        self.target_pitch_deg = 0.0
        self.target_roll_deg = 0.0
        
        # Controls variables
        self.current_target_index = 0 
        self.target_position = self.target_positions[self.current_target_index]
        self.threshold = 0.15  # [m] Threshold for reaching the target
        
        # Changing self.t so that I and D, when going on delta T, go on accurate delta T and not a fixed constant
        self.t = 0.01 #average time between signals in seconds
        self.last_cb_time = self.get_clock().now()

        # Values for rotational (yaw) PID
        self.orientation_quat = [0.0, 0.0, 0.0, 1.0] #current orientation in quaternions
        self.current_orientation = 0.0
        self.target_orientation_quat = [0.0, 0.0, 0.0, 1.0]

        # Setting up relative drone orientation to be zero
        self.drone_rel_zero_orient = False
        self.q0_identity = [0.0, 0.0, 0.0, 1.0] #identity quaternion


        #temp fix: need to rotate drone to face 180 degrees for rigid body then reorient
        
        self.target_orientation = 0.0
        self.k_p_rot = 1.0
        self.k_p_rot_sign = 1

        # ───────────────────────────  CONSTANTS BLOCK  ────────────────────────────
        # --
        # Y constants
        # values for vertical Y (thrust) PID  ── ALTITUDE LOOP (tuned 2025-07-14)
        self.hover       = 41600      # trim thrust to hold level hover
        self.max_thrust  = 56000
        self.min_thrust  = 42000

        self.k_p_y       = 34000+3400      # P-gain
        self.k_i_y       = 800        # I-gain
        self.k_d_y       = 12000      # D-gain

        self.max_yawrate = 15
        self.min_yawrate = -15

        # NEVER CHANGES
        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.int_y_max = 5000 # maximum added thrust from integral component
        # --

        # --
        # X constants
        # values for horizontal X (roll) PID
        # self.k_p_x       = 2.0
        # self.k_i_x       = 0.6
        # self.k_d_x       = 4.1
        
        # NOTE: try increasing kp more so than I. Mostly work with P and D then a little I. P too much = oscillation. I too much = also too much oscillations
        self.k_p_x       = 3.4         
        self.k_i_x       = 0.2   
        self.k_d_x       = 3.4

        self.max_pitch   = 4.0
        self.min_pitch   = -4.0

        # NEVER CHANGES
        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.int_x_max = 3.0 # maximum added pitch from integral component
        # --

        # values for horizontal Z (pitch) PID (negative values because 180 rotation)
        # alligned on the right axis, but roll is now 180 rotation
        self.k_p_z       = -1.2 
        self.k_i_z       = -0.04
        self.k_d_z       = -1.95
        # self.k_p_z       = 0 # was 2
        # self.k_i_z       = 0  # 0.6 
        # self.k_d_z       = 0 #was 4.1   3.0

        self.max_roll = 3.0
        self.min_roll = -3.0 

        # NEVER CHANGES
        self.cur_z_error = 0.0
        self.prev_z_error = 0.0
        self.int_z_error = 0.0
        self.int_z_max = 3.5

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

    def save_pid(self):
        time_s = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") #creates timestamp for every file
        cf2_path = FILE_INITIATION("cf2_path")
        cf2_tuning_name = os.path.basename(cf2_path)
        fname = f"cf2_pid_{cf2_tuning_name[11:30]}.csv" #name of csv
        path = os.path.join(CF2_PID, fname)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
        #print(f"PATHHHHH {path}")
        logger = get_logger("cf_pid_logger")
        logger.info(f"---------------------------------PID WRITE TO {path}")
        #self.get_logger().info(f"[PID] Will write to: {path}")self.yaw_meas), float(self.pitch_meas), float(self.roll_meas
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

        # print(f"[PID] log written to {path}") 
        # logger.info(f"------------------------------[PID] log written to {path}")       

    def listener_callback(self, msg):

        #### Overwriting self.t to be real frequency time for ID terms ######
        now = self.get_clock().now()
        dt = (now - self.last_cb_time).nanoseconds * 1e-9  # seconds
        self.last_cb_time = now

        # Clamp dt to keep the controller sane on the first sample or hiccups
        if not (1e-4 <= dt <= 0.1):          # expected ~100 Hz -> 0.01 s
            dt = self.dt                     # fall back to last dt if crazy
        self.dt = dt
        self.t = dt                          # keep your existing PID code unchanged


        #print(f"SEEESESESES {self.t}") 

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

            # if orientation not zero, set to zero
            if not self.drone_rel_zero_orient:
                self.q0_identity = q_world.inv() 
                self.drone_rel_zero_orient = True
            
            q_rel = q_world * self.q0_identity # relative orientation quaternion
            self.orientation_quat = list(q_rel.as_quat()) # convert to list

            # calls rotational PID function (yawrate)
            yawrate_cmd = self.calculate_yawrate()
            # self.get_logger().info(f"yawrate_cmd = {yawrate_cmd:.2f}")

            # calls X axis PID function (pitch)
            pitch_cmd = self.calculate_pitch()

            # calls Y axis PID function (thrust)
            thrust = self.calculate_thrust()

            # Calls Z axis PID function (roll)
            roll_cmd = self.calculate_roll()

            # create message of type Float Array (all values need to be floats)

            msg = Float32MultiArray()
            msg.data = [float(roll_cmd), float(pitch_cmd), float(yawrate_cmd), float(thrust),      # why is this roll pitch yaw??
                        float(self.position[0]), float(self.position[1]), float(self.position[2]),
                        float(self.yaw_meas), float(self.pitch_meas), float(self.roll_meas) 
                        ]
            # print(msg.data)
            self.pub_commands.publish(msg) #publish commands for drone controller
            # self.get_logger.info()(f"yawrate_cmd--------------------: {msg.data[2]}")
            
            # #This is a timer so the drones stay in one location for a few seconds 
            # if self.is_within_threshold(self.position, self.target_position) and not self.startTimer:
            #     self.startTimer = True
            #     self.startTime = time.time()
            #     #print("Threshold met")
            # else:
            #     self.startTimer = False
            
            # # Check if the drone has reached the target position NEW, stay for 3 seconds 
            # if self.is_within_threshold(self.position, self.target_position) and self.startTimer and time.time() - self.startTime < 3:
            #     self.get_logger().info(f"Reached target position {self.target_position}")
            #     print(f"cf2: Reached target position {self.target_position}")
            #     # Move to the next target if available
            #     self.current_target_index += 1
            #     self.startTime = time.time()
            #     self.startTimer = False
            #     if self.current_target_index < len(self.target_positions):
            #         self.target_position = self.target_positions[self.current_target_index]
            #         self.get_logger().info(f"Moving to next target position {self.target_position}")
            #         print(f"cf2: Moving to next target position {self.target_position}")
            #     else:
            #         self.get_logger().info("All target positions reached. Hovering.")
            #         print("cf2: All target positions reached. Hovering.")
            #         # Optionally, set hover or stop commands

            #new threshold logic
            if self.is_within_threshold(self.position, self.target_position): #if drone is at desired position
                if not self.startTimer: #if the timer for hovering has not started, start it
                    print("cf2 timer started")
                    self.startTimer = True
                    self.startTime = time.time()
                elif time.time() - self.startTime >= 3: #if the drone has been at the desired position for 3 seconds
                    if not self.threshold_met:
                        self.threshold_met = True
                        self.publish_threshold_met()
                        print("cf2: Threshold met")
                    if self.cf1_threshold_met and self.cf1_threshold_met:
                        self.current_target_index += 1
                        if self.current_target_index < len(self.target_positions):  #if these is another target position, move to it
                            self.target_position = self.target_positions[self.current_target_index]
                            self.get_logger().info(f"cf2:Moving to next target position {self.target_position}") 
                            print(f"cf2: moving to next position: {self.target_position}")
                    else:
                        print("Waiting for cf1 to reach threshold.")

                    # else:
                    #     self.get_logger().info("All target positions reached. Hovering.")

        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass

    #helps drone move from one position to the other 
    def is_within_threshold(self, position, target_position):
        # Calculate Euclidean distance between the current position and target position
        dist = np.linalg.norm(np.array(position) - np.array(target_position))
        return dist <= self.threshold
    
    #callback function that gets the threshold_met value from cf1
    def cf1_threshold_met_callback(self, msg):
        self.cf1_threshold_met = msg.data #set the boolean to the value recieved from cf2

    #publishes the threshold_met value to communicate with cf1
    def publish_threshold_met(self):
        threshold_met_msg = Bool()
        threshold_met_msg.data = self.threshold_met
        self.pub_threshold_met.publish(threshold_met_msg)

    # rotational control
    def calculate_yawrate(self):
        # (P term)
        # q_cur = R.from_quat(self.orientation_quat)

        # #####################   READING QUATERNION      ################## 
        # y_cur, w_cur = q_cur.as_quat()[1], q_cur.as_quat()[3]
        # norm = math.hypot(y_cur,w_cur)
        # if norm < 1e-9:
        #     return 0.0
        # else:
        #     angle_y = y_cur / norm
        #     angle_w = w_cur / norm
        #     self.yaw_meas = math.degrees(2.0 * math.atan2(angle_y, angle_w))
        #     self.yaw_meas = (self.yaw_meas + 180) % 360 - 180
        
        # half_angle = np.deg2rad(self.target_orientation)/2.0   #using half angle for p' = qpq*    target orientation is 0...
        # q_des = R.from_quat([0.0, np.sin(half_angle), 0.0, np.cos(half_angle)])   # x,y,z,w 
        # # q_des = R.from_quat([0.0, 0.0, 0.0, 1.0])
        # q_err = q_des * q_cur.inv()
        # y, w = q_err.as_quat()[1], q_err.as_quat()[3]
        # yaw_err = np.rad2deg(2* np.arctan2(y,w)) # second half angle multiplication for full angle
        # yaw_err = (yaw_err + 180) % 360 - 180
        # yawrate_cmd = np.clip(self.k_p_rot * yaw_err, self.min_yawrate, self.max_yawrate)
        
      
        
        # #################################################################

        # self.get_logger.info()(f"CMMMMMMMMDDDDD {yawrate_cmd}")
        #################################################################################################
        # NOTE: this is the previous code for the workaround

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
        ########################################################################################################
        return yawrate_cmd
    
    """# X Axis control
    def calculate_pitch(self):
        
        # (P term)
        self.cur_x_error = self.target_position[0] - self.position[0]
        if -0.01 <= self.cur_x_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
            self.cur_x_error = 0
        x_fp = self.k_p_x * self.cur_x_error
        # print(f"x_fp: {x_fp}")
        
        self.t = time.perf_counter() - self.t
        self.get_logger.info()(f"self_t=============================== {self.t}")
        
        # I term
        #future_int_x_error = self.int_x_error + 0.5 * (self.prev_x_error + self.cur_x_error) * self.t
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

        desired_pitch = x_fp + x_fi + x_fd

        # NEW QUATERNION LOGIC  
        q_cur = R.from_quat(self.orientation_quat)
        half_angle = np.deg2rad(desired_pitch)/2.0   #using half angle for p' = qpq*
        q_des = R.from_quat([np.sin(half_angle), 0.0, 0.0, np.cos(half_angle)])   # x,y,z,w 
        q_err = q_des * q_cur.inv()

        x_value, w_value = q_err.as_quat()[0], q_err.as_quat()[3]
        pitch_err = np.rad2deg(2.0*np.arctan2(x_value, w_value))
        pitch_err = (pitch_err + 180) % 360 - 180
        pitch_cmd = np.clip(desired_pitch + pitch_err, self.min_pitch, self.max_pitch)


        x_cur, w_cur = q_cur.as_quat()[0], q_cur.as_quat()[3]
        norm = math.hypot(x_cur,w_cur)
        if norm < 1e-9:
            return 0.0
        else:
            angle_x = x_cur / norm
            angle_w = w_cur / norm
            self.pitch_meas = math.degrees(2.0 * math.atan2(angle_x, angle_w))   #NOTE: THISSSS IS WHERE THE 180 DEGREE ISSUES COME. CHANGE IF WE EVER NEED TO ROTATE 180 DEGREES
            self.pitch_meas = (self.pitch_meas + 180) % 360 - 180

        return pitch_cmd"""
    
    # FIXED Z axis control 8/19/25
    def calculate_pitch(self):
    
        # set to zero if within margin
        self.cur_z_error = self.target_positions[2] - self.position[2]
        if -0.01 <= self.cur_z_error <= 0.01:
            self.cur_z_error = 0
        
        ######. PID terms for Z-axis position-to-attitude controller  ######

        # (P term)
        z_fp = self.k_p_z * self.cur_z_error # (deg/m) 
        
        # Time interval for I & D terms:
        # self.t = time.perf_counter() - self.t
        # self.get_logger.info()(f"self.t============================ {self.t}")

        # (I term))
        future_int_z_error = self.int_z_error + 0.5 * (self.prev_z_error + self.cur_z_error) * self.t #units of m*s 
        if abs(future_int_z_error * self.k_i_z) < self.int_z_max:
            self.int_z_error = future_int_z_error 
        z_fi = self.k_i_z * self.int_z_error # (to be in degrees Ki_z must be in deg/m*s))
        
        # (D term)
        z_error_dif = self.cur_z_error - self.prev_z_error #m
        z_fd = self.k_d_z * (z_error_dif) / self.t # gain * m/s -> gain= deg/m/s or deg*s/m
        self.prev_z_error = self.cur_z_error # (deg*s/m)
        
        desired_pitch_angle = z_fp + z_fi + z_fd # desired pitch angle in DEGREESSSS
        
        # Desired orientation in quaternion 
        half_angle = np.deg2rad(desired_pitch_angle) / 2.0
        q_desired = R.from_quat([np.sin(half_angle), 0.0, 0.0, np.cos(half_angle)])  # (x,y,z,w)
        # Current orientation in quaternion
        q_current = R.from_quat(self.orientation_quat)
        # orientation error quaternion
        q_error = q_desired * q_current.inv()
        
        # Convert quaternion back to angle
        pitch_error_quat = q_error.as_quat()  
        x_component = pitch_error_quat[0]
        w_component = pitch_error_quat[3]
        pitch_error_angle = np.rad2deg(2.0 * np.arctan2(x_component, w_component))
        pitch_error_angle = (pitch_error_angle + 180) % 360 - 180
        
        pitch_cmd = np.clip(pitch_error_angle, self.min_pitch, self.max_pitch)
        
        ##### current pitch for measurement #####
        x_cur, w_cur = q_current.as_quat()[0], q_current.as_quat()[3]
        norm = math.hypot(x_cur, w_cur)
        if norm > 1e-9:
            angle_x = x_cur / norm
            angle_w = w_cur / norm
            self.pitch_meas = math.degrees(2.0 * math.atan2(angle_x, angle_w))
            self.pitch_meas = (self.pitch_meas + 180) % 360 - 180
        #########################################
        # return pitch_cmd
        return desired_pitch_angle

    # Y axis control 
    def calculate_thrust(self):
        # P term:
        self.cur_y_error = self.target_positions[1]- self.position[1]

        # I term:
        if -0.01 <= self.cur_y_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
            self.cur_y_error = 0

        y_fp = self.k_p_y * self.cur_y_error
        #print(f"y_fp: {y_fp}")

        # self.t = time.perf_counter() - self.t

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
    
    """def calculate_roll(self):
        # P term
        self.cur_z_error = self.target_position[2] - self.position[2]
        z_fp = self.k_p_z * self.cur_z_error
        #print(f"z_fp: {z_fp}")

        self.t = time.perf_counter() - self.t

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
        desired_roll = z_fp + z_fi + z_fd

        #NEW QUATERNION LOGIC
        q_cur = R.from_quat(self.orientation_quat)
        half_angle = np.deg2rad(desired_roll)/2.0   #using half angle for p' = qpq*    target orientation is 0...
        q_des = R.from_quat([0.0, 0.0, np.sin(half_angle), np.cos(half_angle)])   # x,y,z,w 
        q_err = q_des * q_cur.inv()
        
        z_value, w_value = q_err.as_quat()[2], q_err.as_quat()[3]
        roll_err = np.rad2deg(2.0 * np.arctan2(z_value, w_value))
        roll_err = (roll_err + 180) % 360 - 180
        roll_cmd = np.clip(desired_roll + roll_err, self.min_roll, self.max_roll)


        z_cur, w_cur = q_cur.as_quat()[2], q_cur.as_quat()[3]
        norm = math.hypot(z_cur,w_cur)
        if norm < 1e-9:
            return 0.0
        else:
            angle_z = z_cur / norm
            angle_w = w_cur / norm
            self.roll_meas = math.degrees(2.0 * math.atan2(angle_z, angle_w))
            self.roll_meas = (self.roll_meas + 180) % 360 - 180

        # roll_cmd = max(self.min_roll, min(roll_cmd, self.max_roll))     #NOTE: POSSIBLY USE CLIPPING????
        return roll_cmd"""
     # X axis control 
    
    # FIXED X-axis control. 08/19/25
    def calculate_roll(self):

        # set to zero if within margin
        self.cur_x_error = self.target_positions[0] - self.position[0]
        if -0.01 <= self.cur_x_error <= 0.01:
            self.cur_x_error = 0
        # print(f"CUR_X_ERROR: {self.cur_x_error}")

        ######. PID terms for X-axis position-to-attitude controller  ######

        # (P term)
        x_fp = self.k_p_x * self.cur_x_error # (deg/m) 
        # print(f"X_FP: {x_fp}")
        
        # self.t = time.perf_counter() - self.t

        # (I term))
        future_int_x_error = self.int_x_error + 0.5 * (self.prev_x_error + self.cur_x_error) * self.t #units of m*s
        # print(f"future_int_x_error: {future_int_x_error}")
        if abs(future_int_x_error * self.k_i_x) < self.int_x_max:
            self.int_x_error = future_int_x_error 
        x_fi = self.k_i_x * self.int_x_error # (to be in degrees Ki_x must be in deg/m*s))
        # print(f"x_fi: {x_fi}")
        
        # (D term)
        x_error_dif = self.cur_x_error - self.prev_x_error #m
        x_fd = self.k_d_x * (x_error_dif) / self.t # gain * m/s -> gain= deg/m/s or deg*s/m
        self.prev_x_error = self.cur_x_error # (deg*s/m)
        # print(f"x_fd: {x_fd}")
        
        desired_roll_angle = x_fp + x_fi + x_fd # desired roll angle in DEGREESSSS
        
        
        # Desired orientation in quaternion 
        half_angle = np.deg2rad(desired_roll_angle) / 2.0
        q_desired = R.from_quat([0.0, 0.0, np.sin(half_angle), np.cos(half_angle)])  # (x,y,z,w)
        # Current orientation in quaternion
        q_current = R.from_quat(self.orientation_quat)
        # orientation error quaternion
        q_error = q_desired * q_current.inv()
        
        # Convert quaternion back to angle
        roll_error_quat = q_error.as_quat()  
        z_component = roll_error_quat[2]
        w_component = roll_error_quat[3]
        roll_error_angle = np.rad2deg(2.0 * np.arctan2(z_component, w_component))
        roll_error_angle = (roll_error_angle + 180) % 360 - 180
        # print(f"roll_error_angle: {roll_error_angle}")
        
        roll_cmd = np.clip(roll_error_angle, self.min_roll, self.max_roll)
        # roll_cmd = -2
        
        
        ##### current roll for measurement #####
        z_cur, w_cur = q_current.as_quat()[2], q_current.as_quat()[3]
        norm = math.hypot(z_cur, w_cur)
        if norm > 1e-9:
            angle_z = z_cur / norm
            angle_w = w_cur / norm
            self.roll_meas = math.degrees(2.0 * math.atan2(angle_z, angle_w))
            self.roll_meas = (self.roll_meas + 180) % 360 - 180
        #########################################
        # print(f"roll_cmd: {roll_cmd}")
        
        # return roll_cmd

        return desired_roll_angle

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
        #cf2_tuning_static()





      

if __name__ == '__main__':
    main()  



# what's wrong: 2d graph does not show up after cf2 runs
# pid csv date is off because it is created before cf2 tuning is

#original values before I started tuning on july 14, 2025
'''
# values for vertical Y (thrust) PID
        #NOTE: TUNE HERE
        self.hover = 44001 #originally 46500     
        self.max_thrust = 56000 #origionall 50000
        self.min_thrust = 42000
        self.k_p_y = 32000 #previously 15000 on jan 22
        self.k_i_y = 1500 #extra amount of thrust wanted (originally 2000)
        self.k_d_y = 10500
        # self.threshold_met = False

        # values for horizontal X (pitch) PID
        #NOTE: TUNE HERE
        self.k_p_x = 2
        self.k_i_x = 0.6
        self.k_d_x = 4.1
        self.max_pitch = 3.0-1
        self.min_pitch = -3.0

        # values for horizontal Z (roll) PID
        #NOTE:TUNE HERE
        self.k_p_z = 2
        self.k_i_z = 0.6
        self.k_d_z = 4.1 #previously 3.5

        # NEVER CHANGES
        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.int_y_max = 5000 # maximum added thrust from integral component

        # NEVER CHANGES
        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.int_x_max = 3.0 # maximum added pitch from integral component
'''

'''
optitrack stationary cf2 values
how stable?
position:
    x: 1.6873......
    y: 1.149.....
    z: 1.010.....
orientation:
    x: -.028......
    y: -0.869.....
    z: 0.05....
    w: -0.489......
need to calculate error when cf2 is in a stationary location to determine accuracy of optitrack
'''