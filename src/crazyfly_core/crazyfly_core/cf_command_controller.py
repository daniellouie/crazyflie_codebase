#cf_commander for the first drone, number 8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import logging
import time
import cflib
from threading import Thread, Timer
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

import matplotlib.pyplot as plt
import time
import os
from datetime import datetime
import csv
from .optitrack_subscriber2 import OptiTrackSubscriber2
from rclpy.logging import get_logger
#from .flightplots import FILE_INITIATION, cf2_tuning_static
import pandas as pd
import statistics

CF2_PATH = os.path.expanduser("~/crazyfly_ws/flight_navigation_precision/cf2_multiple_waypoint")
#CF1_PATH = os.path.expanduser("~/crazyfly_ws/cf1_tuning_flight_data")
CF1_PATH = os.path.expanduser("~/crazyfly_ws/flight_navigation_precision/cf1_multiple_waypoint")

# the last digit of the radio address specifies which drone its connected (currently either 7 or 8)
#      ----------     NOTE: CHANGE "address" last value to:          ----------
#      ----------                                       cf1: 8       ----------
#      ----------                                       cf2: 7       ----------
# address = 'radio://0/80/2M/E7E7E7E7E8'  # cf1
address = 'radio://0/80/2M/E7E7E7E7E7'  # cf2
if address[-1] == '8':
    CF_PATH = CF1_PATH
else:
    CF_PATH = CF2_PATH

link_uri = uri_helper.uri_from_env(default=address)

# test

logging.basicConfig(level=logging.ERROR)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('cf_driver')

        self.subscriptionss = []
        self.subscriptionss.append(self.create_subscription(
            Float32MultiArray,
            '/cf1/commands',
            self.listener_callback,
            10))
        self.subscriptionss.append(self.create_subscription(
            Float32MultiArray,
            '/cf1/controller_pid_details',
            self.controller_pid_details_callback,
            10
        ))

        cflib.crtp.init_drivers()


        #Crazyflie drone 1 (number 8) connection
        self._cf1 = Crazyflie(rw_cache='./cache')
        self._cf1.connected.add_callback(self._connected)
        self._cf1.disconnected.add_callback(self._disconnected)
        self._cf1.connection_failed.add_callback(self._connection_failed)
        self._cf1.connection_lost.add_callback(self._connection_lost)


        # Flight variables for both drones
        self.roll1, self.pitch1, self.yawrate1, self.thrust1 = 0.0, 0.0, 0.0, 0.0
       
        # Position variables
        self.x_position1, self.y_position1, self.z_position1 = 0.0, 0.0, 0.0
        self.cf1_position = []

        self.command_roll_log = []
        self.command_pitch_log = []
        self.command_thrust_log = []
        self.command_yawrate_log = []

        self.cf2_command_values = []

       
        """ TTTTTTTTTTTTTTTTTTTTTTTTIMMMMMMMMMMMMMMMMMMMMMEEEEEEEEEEEEEEEEEEEEEEEE"""
        # limit flight time for testing
        self.flight_duration = 25 # 20.0 #in seconds
        """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
        # constant command values for testing
        self.const_thrust = 44000
        self.const_roll = 0 #-3,3 range
        self.const_pitch = 0 #-3,3 range
        self.const_yawrate = 0 #-15,15 range

        self._cf1.open_link(link_uri)
       
       

        #send initial command of zeros (needed for crazyflie protocol)
        self._cf1.commander.send_setpoint(0, 0, 0, 0)
       

        # Graphing and data collection
        self.timestamp_data = []
        self.x_log = []
        self.y_log = []
        self.z_log = []

        self.yaw_log = []
        self.pitch_log = []
        self.roll_log = []

        self.error_x_log = []
        self.error_y_log = []
        self.error_z_log = []

        self.command_roll_log = []
        self.command_pitch_log = []
        self.command_yaw_log = []
        self.command_thrust_log = []

        self.command_pitch_p_log = []
        self.command_pitch_i_log = []
        self.command_pitch_d_log = []
        self.command_roll_p_log = []
        self.command_roll_i_log = []
        self.command_roll_d_log = []
        self.command_thrust_p_log = []
        self.command_thrust_i_log = []
        self.command_thrust_d_log = []

        self.tracking_time = []

        self.thrust_data = []
        self.y_fp_data = []
        self.y_fi_data = []

        self.start_time = time.time()

    # this function is called each time a 'command' message is received
    def listener_callback(self, msg):
        # ensure commands for all axis are recieved
        if len(msg.data) >= 7:
            self.roll1 =    msg.data[0]
            self.pitch1 =   msg.data[1]
            self.yawrate1 = msg.data[2]
            self.thrust1 =  int(msg.data[3]) #thrust needs to be an int

            self.x_position1 = msg.data[4]
            self.y_position1 = msg.data[5]
            self.z_position1 = msg.data[6]

            # record data for graphing
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            self.timestamp_data.append(elapsed_time)
            self.x_log.append(self.x_position1)
            self.y_log.append(self.y_position1)
            self.z_log.append(self.z_position1)

            self.yaw_log.append(msg.data[7])
            self.pitch_log.append(msg.data[8])
            self.roll_log.append(msg.data[9])

            self.error_x_log.append(msg.data[10])
            self.error_y_log.append(msg.data[11])
            self.error_z_log.append(msg.data[12])

            self.command_roll_log.append(self.roll1)
            self.command_pitch_log.append(self.pitch1)
            self.command_thrust_log.append(self.thrust1)
            self.command_yawrate_log.append(self.yawrate1)

            # self.cur_yaw_data.append(self.yaw_meas)
            # self.cur_pitch_data.append(self.pitch_meas)
            # self.cur_roll_data.append(self.roll_meas)

            self.thrust_data.append(self.thrust1)
            self.tracking_time.append(datetime.now().strftime("%Y%m%d_%H%M%S.%f"))

            # self.y_fp_data.append(y_fp)
            # self.y_fp_data.append(y_fi)
        else:
            print("Error: incorrect msg length")

        self.run_motors() #call function to send commands to crazyflie
       
       
    # function to run motors called each time data is recieved
    def run_motors(self):
        #comment these out to run REAL values
        # self.roll = self.const_roll
        # self.pitch = self.const_pitch
        # self.yawrate = self.const_yawrate
        # self.thrust = self.const_thrust

        # Send commands to drone 1
        self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, self.thrust1)
        print(f"Drone 1: Roll = {self.roll1}, Pitch = {self.pitch1}, Yawrate = {self.yawrate1}, Thrust = {self.thrust1}")


        #self._cf.commander.send_setpoint(self.const_roll, self.const_pitch, self.const_yawrate, self.const_thrust)

    def controller_pid_details_callback(self, msg):
        self.command_pitch_p_log.append(msg.data[0])
        self.command_pitch_i_log.append(msg.data[1])
        self.command_pitch_d_log.append(msg.data[2])
        self.command_roll_p_log.append(msg.data[3])
        self.command_roll_i_log.append(msg.data[4])
        self.command_roll_d_log.append(msg.data[5])
        self.command_thrust_p_log.append(msg.data[6])
        self.command_thrust_i_log.append(msg.data[7])
        self.command_thrust_d_log.append(msg.data[8])

    # Unused function that uses Threading
    def _send_thrust_command(self):
       
        while True:
            self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, self.thrust1)
            time.sleep(0.05)  # Send commands at 20Hz

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print(f"CONNECTED TO {link_uri}")
        # Start a separate thread to continuously send thrust commands.
        #temporatily commented out for debugging (not working)
        #Thread(target=self._send_thrust_command).start()

    # add a ramp down for safety
    def _stop_program(self):
        """Stops the Crazyflie and exits the program."""
        print("Timeout reached, stopping the Crazyflie.")

        # set to the current thrust value
        rampdown_thrust1 = self.thrust1
        rclpy.shutdown()  # Shutdown ROS 2 before ramping down to prevent override


        # ramp down thrust until reaching threshold to cut power, ideally on the ground
        while rampdown_thrust1 > 42000:
            if rampdown_thrust1 > 42000:
                rampdown_thrust1 -= 150
            print(f"running ramp down: {rampdown_thrust1}")
            self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, rampdown_thrust1)
            time.sleep(0.1)
        self._cf1.commander.send_setpoint(0, 0, 0, 0)  # Stop the motors
        self._cf1.close_link()  # Disconnect from the Crazyflie
        rclpy.shutdown() #gpt recommended code, temporary

       


    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
    '''
    time_s = datetime.now().strftime("%Y%m%d_%H%M%S") #creates timestamp for every file
        fname = f"cluster_dot_{time_s}.csv" #name of csv
        path = os.path.join(LOG_DIR, fname)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
        with open(path, "w", newline="") as file:
            w = csv.writer(file)
            w.writerow(       # header row
                ["time_s",
                "x1dot","y1dot","z1dot",
                "x2dot","y2dot","z2dot"])
            w.writerows(self.cluster_dot)  #cluster values
        print(f"[Cluster] log written to {path}")'''



    def save_data(self):
        time_s = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") #creates timestamp for every file
        if address[-1] == '8':
            fname = f"cf1_tuning_{time_s}.csv" #name of csv
        else:
            fname = f"cf2_tuning_{time_s}.csv" #name of csv    
        #fname = f"optitrack_test_{time_s}.csv"
        path = os.path.join(CF_PATH, fname)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
        self.cf1_position.append([datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f"), self.x_log, self.y_log, self.z_log])
        with open(path, "w", newline="") as file:
            w = csv.writer(file)
            w.writerow(       # header row
                ["time_s",
                "x","y","z",
                ])
           
            for (t, x, y, z) in zip(self.tracking_time, self.x_log, self.y_log, self.z_log):
                w.writerow([t, x, y, z])



    #----- Saving Command Values & Thrust Values to Optimize Tuning -----#
    def save_all_values(self):
        time_s = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") #creates timestamp for every file
        fname = f"cf1_all_values_{time_s}.csv" #name of csv
        if address[-1] == '8':
            CF_COMMAND_PATH = os.path.expanduser("~/crazyfly_ws/flight_navigation_precision/command_values_for_stability/cf1_command_values")  # if radio signal ends in 7 (cf2)
            # self.cf1_command_values.append([time_s, self.cf1_command_roll, self.cf1_command_pitch, self.cf1_command_yaw, self.cf1_command_thrust]) #msg.data order = roll, pitch, yaw, thrust
        else:
            CF_COMMAND_PATH = os.path.expanduser("~/crazyfly_ws/flight_navigation_precision/command_values_for_stability/cf2_command_values") # else its cf2
            # self.cf2_command_values.append([time_s, self.roll1, self.pitch1, self.yawrate1, self.thrust1]) #msg.data order = roll, pitch, yaw, thrust
        path = os.path.join(CF_COMMAND_PATH, fname)
        self.get_logger().info(path)
        #self.get_logger().info("path: " + path)
        with open(path, "w", newline="") as file:
            w = csv.writer(file)
            w.writerow(       # header row
                ["time_s","x","y","z","x_error","y_error","z_error","yaw","pitch","roll",
                 "yaw_command","pitch_command","roll_command","thrust_command",
                 "pitch_cmd_p","pitch_cmd_i","pitch_cmd_d","roll_cmd_p","roll_cmd_i","roll_cmd_d","thrust_cmd_p","thrust_cmd_i",
                 "thrust_cmd_d"
                ])
            for i in range(len(self.x_log)):
                w.writerow([self.tracking_time[i], self.x_log[i], self.y_log[i], self.z_log[i],
                            self.error_x_log[i], self.error_y_log[i], self.error_z_log[i],
                            self.yaw_log[i], self.pitch_log[i], self.roll_log[i],
                            self.command_yawrate_log[i], self.command_pitch_log[i], self.command_roll_log[i], self.command_thrust_log[i],
                            self.command_pitch_p_log[i], self.command_pitch_i_log[i], self.command_pitch_d_log[i],
                            self.command_roll_p_log[i], self.command_roll_i_log[i], self.command_roll_d_log[i],
                            self.command_thrust_p_log[i], self.command_thrust_i_log[i], self.command_thrust_d_log[i]])
    #---------------------------------------------------------------------#


# function to save optitrack data for accuracy tracking
    """# def save_data_optitrack(self):
    #     time_s = datetime.now().strftime("%Y-%m-%d_%H:%M:%S") #creates timestamp for every file
    #     fname = f"optitrack_test_cf2_{time_s}.csv" #name of csv
    #     path = os.path.join(os.path.expanduser("~/crazyfly_ws/optitrack_data_raw"), fname)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
    #     self.cf1_position.append([datetime.now().strftime("%Y-%m-%d_%H:%M:%S"), self.cur_x_data, self.cur_y_data, self.cur_z_data, self.cur_yaw_data, self.cur_pitch_data, self.cur_roll_data])
    #     with open(path, "w", newline="") as file:
    #         w = csv.writer(file)
    #         w.writerow(       # header row
    #             ["time_s",
    #             "x","y","z", "yaw", "pitch", "roll"
    #             ])
           
    #         for (t, x, y, z, yaw, pitch, roll) in zip(self.tracking_time, self.cur_x_data, self.cur_y_data, self.cur_z_data, self.cur_yaw_data, self.cur_pitch_data, self.cur_roll_data):
    #             w.writerow([t, x, y, z, yaw, pitch, roll])
    #     df = pd.read_csv(path)
    #     values_x = list(df['x'])
    #     values_y = list(df['y'])
    #     values_z = list(df['z'])
    #     mean_x = statistics.mean(values_x)
    #     mean_y = statistics.mean(values_y)
    #     mean_z = statistics.mean(values_z)
    #     std_x = statistics.stdev(values_x)
    #     std_y = statistics.stdev(values_y)
    #     std_z = statistics.stdev(values_z)

    #     values_yaw = list(df['yaw'])
    #     values_pitch = list(df['pitch'])
    #     values_roll = list(df['roll'])

    #     mean_yaw = statistics.mean(values_yaw)
    #     mean_pitch = statistics.mean(values_pitch)
    #     mean_roll = statistics.mean(values_roll)

    #     std_yaw = statistics.stdev(values_yaw)
    #     std_pitch = statistics.stdev(values_pitch)
    #     std_roll = statistics.stdev(values_roll)



    #     self.get_logger().info(f"MEAAAAAAAN OF  x {mean_x}")
    #     self.get_logger().info(f"MEAAAAAAAN OF  y {mean_y}")
    #     self.get_logger().info(f"MEAAAAAAAN OF  z {mean_z}")
    #     self.get_logger().info(f"SSSSSSSSTANDARD DEVIATION OF  x {std_x}")
    #     self.get_logger().info(f"SSSSSSSSTANDARD DEVIATION OF  y {std_y}")
    #     self.get_logger().info(f"SSSSSSSSTANDARD DEVIATION OF  z {std_z}")

    #     fname2 = f"optitrack_data_cf2_{time_s}.csv" #name of csv
    #     path2 = os.path.join(os.path.expanduser("~/crazyfly_ws/optitrack_data_stats"), fname2)             # file ends up here where LOG_DIR is the I_Joc_values folder or directory
    #     with open(path2, "w", newline="") as file:
    #         w = csv.writer(file)
    #         w.writerow(       # header row
    #             ["x", "z", "y", "mean_x", "mean_y", "mean_z", "mean_yaw", "mean_pitch", "mean_roll", "std_x", "std_y", "std_z", "std_yaw", "std_pitch", "std_roll"
    #             ])
           
    #         w.writerow([round(x*2)/2, round(z*2)/2, round(y*2)/2, mean_x, mean_y, mean_z, mean_yaw, mean_pitch, mean_roll, std_x, std_y, std_z, std_yaw, std_pitch, std_roll])

    #     # make new csv with mean, std, max, min, range

    #     plt.figure()
    #     plt.subplot(2,2,1)
    #     plt.plot(path['time_s'], path['x'], 'r-', label='X Position')
    #     plt.xlabel('Time(s)')
    #     plt.ylabel('X Position (m)')
    #     plt.title('X Position Over Time')
    #     plt.ylim(bottom=0)
    #     plt.legend()


    #     plt.subplot(2,2,2)
    #     plt.plot(path['time_s'], path['y'], 'r-', label='Y Position')
    #     plt.xlabel('Time(s)')
    #     plt.ylabel('Y Position (m)')
    #     plt.title('Y Position Over Time')
    #     plt.ylim(bottom=0)
    #     plt.legend()


    #     plt.subplot(2,2,3)
    #     plt.plot(path['time_s'], path['z'], 'r-', label='Z Position')
    #     plt.xlabel('Time(s)')
    #     plt.ylabel('Z Position (m)')
    #     plt.title('Z Position Over Time')
    #     plt.ylim(bottom=0)
    #     plt.legend()
    #     plt.show()"""



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Set up a timer to stop the program after set amount of time (in seconds)
    timeout_timer = Timer(minimal_subscriber.flight_duration, minimal_subscriber._stop_program)
    timeout_timer.start()

    rclpy.spin(minimal_subscriber)

    # minimal_subscriber._cf.close_link()  # Disconnect from the Crazyflie
    # Plotting Graph one
    # Currently plotting Y vs time, X vs time, Z vs time and Thrust vs time
    plt.figure()
    plt.subplot(2,2,1)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.y_log, 'r-', label='Y Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Y Position (m)')
    plt.title('Y Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,2)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.x_log, 'r-', label='X Position')
    plt.xlabel('Time(s)')
    plt.ylabel('X Position (m)')
    plt.title('X Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,3)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.z_log, 'r-', label='Z Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Z Position (m)')
    plt.title('Z Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    # graphing thrust over time
    plt.subplot(2,2,4)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.thrust_data, 'b-', label='Thrust')
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust')
    plt.title('Thrust Over Time')
    plt.legend()
    plt.show()
    print("Plotted")

    minimal_subscriber.save_data()
    #minimal_subscriber.save_data_optitrack()
    #cf2_tuning_static()

    print("\n\n\n**********************Hello****************\n\n\n")
    minimal_subscriber.save_all_values()    # this saves commanded values for optimizing flights 
    
   

    minimal_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

