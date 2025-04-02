import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import logging
import time
import cflib
from threading import Thread, Timer
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

link_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('cf_driver')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cf1/commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        cflib.crtp.init_drivers()

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.pitch = 0.0
        self.roll = 0.0
        self.yawrate = 0.0
        self.cf_th = 0.0

        self._cf.open_link(link_uri)

        self._cf.commander.send_setpoint(0, 0, 0, 0) 

        print('Connecting to %s' % link_uri)

    def listener_callback(self, msg):
        if len(msg.data) >=4:
            self.roll = msg.data[0]
            self.pitch = msg.data[1]
            self.yawrate = msg.data[2]
            self.cf_th = int(msg.data[3]) #thrust needs to be an int
            
            print(f"Received: Roll = {self.roll}, Pitch = {self.pitch}, Yawrate = {self.yawrate}, Thrust = {self.cf_th}")
        self.run_motors()


    # Current function to run motors called each time data is recieved
    def run_motors(self):
        #print(f"run_motors:{self.cf_th}")
        const_thrust = 45000
        const_roll = 0 #-3,3 range
        const_pitch = 0 #-3,3 range
        const_yawrate = 0 #-15,15 range
        self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.cf_th)


        #self._cf.commander.send_setpoint(const_roll, const_pitch, const_yawrate, const_thrust)
        #time.sleep(0.05)  # Send commands at 20Hz

    # Unused function that uses Threading
    def _send_thrust_command(self):
        # Unlock startup thrust protectionc
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)  # Give it a brief moment to register the unlock command

        while True:
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.cf_th)
            
            time.sleep(0.05)  # Send commands at 20Hz

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to continuously send thrust commands.
        #temporatily commented out for debugging (not working)
        #Thread(target=self._send_thrust_command).start()

    # add a ramp down for safety
    def _stop_program(self):
        """Stops the Crazyflie and exits the program."""
        print("Timeout reached, stopping the Crazyflie.")
        thrust_mult = 1 #origionally 1 
        thrust_step = 500 #origional 500 
        thrust = 34000 #origional 30000


        while thrust >= 30000: #note, set to 40000 it flew to the roof 
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 35000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)  # Stop the motors
        self._cf.close_link()  # Disconnect from the Crazyflie
        rclpy.shutdown()  # Shutdown ROS 2

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

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Set up a timer to stop the program after 15 seconds
    timeout_timer = Timer(30.0, minimal_subscriber._stop_program)
    timeout_timer.start()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

