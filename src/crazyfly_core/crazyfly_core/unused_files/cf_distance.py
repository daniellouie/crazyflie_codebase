
import time
import logging 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

# Initialize the low-level drivers 
cflib.crtp.init_drivers()

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

logging.basicConfig(level=logging.ERROR)

# Create a Crazyflie object
cf = Crazyflie(rw_cache='./cache')

# Function to calculate thrust based on desired height
def calculate_thrust(current_height, desired_height):

    # Simple proportional controller
    k_p = 10000  # Proportional gain
    error = desired_height - current_height
    thrust = 37000 + k_p * error
    
    # Clamp thrust to valid values (37000 is a rough hover thrust value)
    thrust = max(20000, min(thrust, 50000))
    
    return int(thrust)

# Connect to the Crazyflie
cf.open_link(uri)

# Wait for Crazyflie to be ready
time.sleep(1)

# Assuming the MotionCommander is used for higher-level control
with MotionCommander(cf) as mc:
    desired_height = 0.5  # Desired height in meters

    start_time = time.time()
    while time.time() - start_time < 10:  # Run for 20 seconds
        # current_height = mc._get_z_distance()  # Get current height (replace with your sensor data)
        current_height = 0.4

        # Calculate the necessary thrust
        #thrust = calculate_thrust(current_height, desired_height)
        thrust = 37000

        
        # Send the calculated thrust to the Crazyflie
        cf.commander.send_setpoint(0, 0, 0, thrust)
        
        time.sleep(0.05)  # Control loop at 20 Hz

# Make sure to stop the Crazyflie when done
cf.commander.send_setpoint(0, 0, 0, 0)
time.sleep(0.1)
cf.close_link()


'''
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import logging
import cflib
from cflib.utils import uri_helper

# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)

# URI to the Crazyflie to connect to
#uri = 'radio://0/80/2M/E7E7E7E7E7'
link_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Create a Crazyflie object
cf = Crazyflie(rw_cache='./cache')

# Connect to the Crazyflie
cf.open_link(link_uri)

# Function to calculate thrust based on desired height
def calculate_thrust(current_height, desired_height):
   

    # Simple proportional controller
    k_p = 2000  # Proportional gain
    error = desired_height - current_height
    thrust = 37000 + k_p * error
    print("thrust: %s" % thrust)
    # Clamp thrust to valid values (37000 is a rough hover thrust value)
    thrust = max(10000, min(thrust, 45000))


    thrust = 30000
    print("thrust in func: %s" % thrust)
    return int(thrust)

try:
    # Wait for Crazyflie to be ready
    time.sleep(5)

    # Assuming the MotionCommander is used for higher-level control
    with MotionCommander(cf) as mc:
        desired_height = 0.5  # Desired height in meters
        
        start_time = time.time()
        while time.time() - start_time < 10:  # Run for 20 seconds
            # current_height = mc._get_z_distance()  # Get current height (replace with your sensor data)
            current_height = 0.4
            # Calculate the necessary thrust
            thrust = calculate_thrust(current_height, desired_height)

            print("thrust in main: %s" % thrust)
            # Send the calculated thrust to the Crazyflie
            cf.commander.send_setpoint(0, 0, 0, thrust)
            
            time.sleep(0.01)  # Control loop at 100 Hz
finally:
    # Make sure to stop the Crazyflie when done
    cf.commander.send_stop_setpoint()
    cf.close_link()
'''