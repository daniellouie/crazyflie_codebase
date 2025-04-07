# crazyflie_control.py
import rclpy
import logging
import sys
import time
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

#test

from optitrack_subscriber import OptiTrackSubscriber  # Import the subscriber class


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
logging.basicConfig(level=logging.ERROR)


# Desired position (replace with your desired coordinates)
DESIRED_POSITION = [1.06, 1.8, 0.72]  # Example desired position in meters

def fly_to_position_test(opti_track_node, desired_position):
    while rclpy.ok():
        current_position = opti_track_node.get_position()
        x, y, z = current_position
        dx, dy, dz = desired_position


        # Check if we have reached the desired position
        if abs(x - dx) < 0.05 and abs(y - dy) < 0.05 and abs(z - dz) < 0.05:
            print("Reached desired position")
            break
        
        y_error = dy-y
        print(f"y_error: {y_error: .2f}")

        thrust = calculate_thrust(y_error)

        print(f"thrust: {thrust}")

        # Send position setpoint to move towards the desired position
        #print(f"Moving towards position: {desired_position}, Current position: {current_position}")


        # Allow ROS 2 to process callbacks
        rclpy.spin_once(opti_track_node, timeout_sec=0.1)


        time.sleep(0.1)  # Adjust as needed for smoother flight


# Function to calculate thrust based on desired height
def calculate_thrust(y_error):

    # Simple proportional controller
    k_p = 10000  # Proportional gain
    error = y_error
    thrust = 35000 + k_p * error
    
    # Clamp thrust to valid values (37000 is a rough hover thrust value)
    thrust = max(30000, min(thrust, 40000))
    
    return int(thrust)

# def fly_to_position(scf, opti_track_node, desired_position):
#     with Commander(scf) as commander:
#         while rclpy.ok():
#             current_position = opti_track_node.get_position()
#             x, y, z = current_position
#             dx, dy, dz = desired_position


#             # Check if we have reached the desired position
#             if abs(x - dx) < 0.05 and abs(y - dy) < 0.05 and abs(z - dz) < 0.05:
#                 print("Reached desired position")
#                 commander.send_position_setpoint(dx, dy, dz, 0)
#                 commander.send_stop_setpoint()  # Send stop command to stabilize the drone
#                 break


#             # Send position setpoint to move towards the desired position
#             commander.send_position_setpoint(dx, dy, dz, 0)
#             print(f"Moving towards position: {desired_position}, Current position: {current_position}")


#             # Allow ROS 2 to process callbacks
#             rclpy.spin_once(opti_track_node, timeout_sec=0.1)


#             time.sleep(0.1)  # Adjust as needed for smoother flight


def main():
    # Initialize ROS 2
    rclpy.init(args=sys.argv)


    # Create the OptiTrack subscriber node
    opti_track_node = OptiTrackSubscriber()


    # Initialize Crazyflie drivers
    init_drivers()

    fly_to_position_test(opti_track_node, DESIRED_POSITION)

    # Connect to Crazyflie
    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    #     print("Connected to Crazyflie")


    #     # Fly to the desired position
    #     fly_to_position_test(scf, opti_track_node, DESIRED_POSITION)


    # Shutdown ROS 2 after flight is complete
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# # crazyflie_control.py
# import rclpy

# import logging
# import sys
# import time
# from cflib.crtp import init_drivers
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.commander import Commander
# from cflib.utils import uri_helper

# from optitrack_subscriber import OptiTrackSubscriber  # Import the subscriber class

# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
# logging.basicConfig(level=logging.ERROR)

# # Desired position (replace with your desired coordinates)
# DESIRED_POSITION = [1.06, 0.16, 0.72]  # Example desired position in meters

# def fly_to_position(scf, current_position, desired_position):
#     with Commander(scf) as commander:
#         while True:
#             x, y, z = current_position
#             dx, dy, dz = desired_position
            
#             if abs(x - dx) < 0.05 and abs(y - dy) < 0.05 and abs(z - dz) < 0.05:
#                 print("Reached desired position")
#                 commander.send_position_setpoint(dx, dy, dz, 0)
#                 break

#             print("looking")
#             current_position = opti_track_node.get_position()
#             print(current_position)

#             commander.send_position_setpoint(dx, dy, dz, 0)
#             time.sleep(0.1)

# def fly_to_position_test(desired_position):
#     while True:
#         current_position = opti_track_node.get_position()
#         print(current_position)
#         x,y,z = current_position

#         dx, dy, dz = desired_position

        
#         if abs(x - dx) < 0.05 and abs(y - dy) < 0.05 and abs(z - dz) < 0.05:
#             print("Reached desired position")
#             #commander.send_position_setpoint(dx, dy, dz, 0)
#             break

#         print("looking")
        
#         #commander.send_position_setpoint(dx, dy, dz, 0)
#         time.sleep(0.1)

# if __name__ == '__main__':
#     # Initialize ROS 2
#     rclpy.init(args=sys.argv)

#     # Create the OptiTrack subscriber node
#     opti_track_node = OptiTrackSubscriber()

#     # Initialize Crazyflie drivers
#     init_drivers()

#     # Connect to Crazyflie
#     with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
#         print("Connected to Crazyflie")

#         # Start the ROS 2 loop
#         rclpy.spin_once(opti_track_node, timeout_sec=1)
        
#         while rclpy.ok():
#             # Update current position from OptiTrack
#             # current_position = opti_track_node.get_position()
#             # print(current_position)

#             # Fly to desired position (commented out so the drone doesnt attack us)
#             # fly_to_position(scf, current_position, DESIRED_POSITION)
#             fly_to_position_test(DESIRED_POSITION)

            
#             # Continue to process ROS 2 callbacks
#             rclpy.spin_once(opti_track_node, timeout_sec=0.1)

