import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16

import logging
import time
import cflib
from threading import Thread
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

link_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('cf_driver')
        self.subscription = self.create_subscription(
            UInt16,
            '/cf1/thrust',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


        cflib.crtp.init_drivers()

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.pitch = 0
        self.roll = 0
        self.yawrate = 0
        self.cf_th = 0

        self._cf.open_link(link_uri)

        self._cf.commander.send_setpoint(0, 0, 0, 0)

        
        print('Connecting to %s' % link_uri)


    def listener_callback(self, msg):
        self.cf_th = msg.data
        print(f"listener_callback thrust: {self.cf_th}")
        #self._ramp_motors
        

    def _ramp_motors(self):
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.01)
        #continuosly loops at 20Hz
        while True:
            print(f"ramp_motors thrust: {self.cf_th}")
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.cf_th)
            time.sleep(0.05)
        # thrust_mult = 1 #origionally 1 
        # thrust_step = 500 #origional 500 
        # thrust = 30000 #origional 30000
        # pitch = 0 #origionally 0 
        # roll = 0
        # yawrate = 0

        # # Unlock startup thrust protection
        # self._cf.commander.send_setpoint(0, 0, 0, 0)

        # while thrust >= 30000: #note, set to 40000 it flew to the roof 
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     time.sleep(0.1)
        #     if thrust >= 35000:
        #         thrust_mult = -1
        #     thrust += thrust_step * thrust_mult
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # # Make sure that the last packet leaves before the link is closed
        # # since the message queue is not flushed before closing
        # time.sleep(0.1)
        # self._cf.close_link()


    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
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

    cflib.crtp.init_drivers()

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()