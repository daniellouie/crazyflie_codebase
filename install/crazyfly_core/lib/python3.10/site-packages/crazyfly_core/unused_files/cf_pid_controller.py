import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R

# this class is the PID controller for each drone
# it takes in the drone name and the PID parameters (determined by input)
# this approach is more modular for multiple drones
class DronePIDController(Node):
    def __init__(self, drone_id, pid_params):
        super().__init__(f'drone_pid_controller_{drone_id}')

        self.drone_id = drone_id

        # PID parameters (passed during initialization)
        self.k_p_x = pid_params['k_p_x']
        self.k_i_x = pid_params['k_i_x']
        self.k_d_x = pid_params['k_d_x']
        self.k_p_y = pid_params['k_p_y']
        self.k_i_y = pid_params['k_i_y']
        self.k_d_y = pid_params['k_d_y']
        self.k_p_z = pid_params['k_p_z']
        self.k_i_z = pid_params['k_i_z']
        self.k_d_z = pid_params['k_d_z']
        self.k_p_rot = pid_params['k_p_rot']

        # Limits
        self.max_pitch = pid_params['max_pitch']
        self.min_pitch = pid_params['min_pitch']
        self.max_roll = pid_params['max_roll']
        self.min_roll = pid_params['min_roll']
        self.max_thrust = pid_params['max_thrust']
        self.min_thrust = pid_params['min_thrust']
        self.max_yawrate = pid_params['max_yawrate']
        self.min_yawrate = pid_params['min_yawrate']

        # Hover thrust
        self.hover = pid_params['hover']

        # PID state variables
        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.cur_z_error = 0.0
        self.prev_z_error = 0.0
        self.int_z_error = 0.0

        # Time step
        self.t = 0.01

        # Current and commanded positions
        self.current_position = [0.0, 0.0, 0.0]
        self.commanded_position = [0.0, 0.0, 0.0]

        # Orientation
        self.orientation_quat = [0.0, 0.0, 0.0, 0.0]
        self.current_orientation = 0.0
        self.target_orientation = 90  # Target yaw in degrees

        # Subscribers
        self.create_subscription(
            PoseStamped,
            f'/vrpn_mocap/{drone_id}/pose', # current position from optitrack
            self.current_position_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            f'/{drone_id}_cmd', # commanded position from cluster controller
            self.commanded_position_callback,
            10
        )

        # Publisher for motor commands
        self.motor_command_publisher = self.create_publisher(
            Float32MultiArray,
            f'/cf{drone_id[-1]}/motor_commands',
            10
        )

    def current_position_callback(self, msg):
        """Callback to update the current position and orientation of the drone."""
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z

        self.orientation_quat[0] = msg.pose.orientation.x
        self.orientation_quat[1] = msg.pose.orientation.y
        self.orientation_quat[2] = msg.pose.orientation.z
        self.orientation_quat[3] = msg.pose.orientation.w

        self.calculate_motor_commands()

    def commanded_position_callback(self, msg):
        """Callback to update the commanded position of the drone."""
        self.commanded_position[0] = msg.pose.position.x
        self.commanded_position[1] = msg.pose.position.y
        self.commanded_position[2] = msg.pose.position.z

    def calculate_motor_commands(self):
        """Calculate motor commands using PID controllers."""
        # Calculate yawrate
        yawrate = self.calculate_yawrate()

        # Calculate pitch (X-axis control)
        pitch = self.calculate_pitch()

        # Calculate thrust (Y-axis control)
        thrust = self.calculate_thrust()

        # Calculate roll (Z-axis control)
        roll = self.calculate_roll()

        # Publish motor commands
        motor_command_msg = Float32MultiArray()
        motor_command_msg.data = [roll, pitch, yawrate, thrust]
        self.motor_command_publisher.publish(motor_command_msg)

    def calculate_yawrate(self):
        """Calculate yawrate using a P controller."""
        r = R.from_quat(self.orientation_quat)
        _, yaw, _ = r.as_euler('xyz', degrees=True)
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

    def calculate_thrust(self):
        """Calculate thrust using a PID controller."""
        self.cur_y_error = self.commanded_position[1] - self.current_position[1]
        y_fp = self.k_p_y * self.cur_y_error
        y_fi = self.k_i_y * self.int_y_error
        y_fd = self.k_d_y * (self.cur_y_error - self.prev_y_error) / self.t

        self.int_y_error += self.cur_y_error * self.t
        self.prev_y_error = self.cur_y_error

        thrust = self.hover + y_fp + y_fi + y_fd
        thrust = int(max(self.min_thrust, min(thrust, self.max_thrust)))
        return thrust

    def calculate_roll(self):
        """Calculate roll using a PID controller."""
        self.cur_z_error = self.commanded_position[2] - self.current_position[2]
        z_fp = self.k_p_z * self.cur_z_error
        z_fi = self.k_i_z * self.int_z_error
        z_fd = self.k_d_z * (self.cur_z_error - self.prev_z_error) / self.t

        self.int_z_error += self.cur_z_error * self.t
        self.prev_z_error = self.cur_z_error

        roll = z_fp + z_fi + z_fd
        roll = max(self.min_roll, min(roll, self.max_roll))
        return roll


def main(args=None):
    rclpy.init(args=args)

    # PID parameters for each drone
    pid_params_cf1 = {
        'k_p_x': 2.0, 'k_i_x': 0.6, 'k_d_x': 4.0,
        'k_p_y': 15000, 'k_i_y': 3250, 'k_d_y': 10000,
        'k_p_z': 2.0, 'k_i_z': 0.6, 'k_d_z': 3.5,
        'k_p_rot': 0.25,
        'max_pitch': 3.0, 'min_pitch': -3.0,
        'max_roll': 3.0, 'min_roll': -3.0,
        'max_thrust': 55000, 'min_thrust': 42000,
        'max_yawrate': 15, 'min_yawrate': -15,
        'hover': 44000
    }

    pid_params_cf2 = {
        'k_p_x': 1.8, 'k_i_x': 0.5, 'k_d_x': 3.8,
        'k_p_y': 14000, 'k_i_y': 3000, 'k_d_y': 9500,
        'k_p_z': 1.9, 'k_i_z': 0.5, 'k_d_z': 3.2,
        'k_p_rot': 0.22,
        'max_pitch': 3.0, 'min_pitch': -3.0,
        'max_roll': 3.0, 'min_roll': -3.0,
        'max_thrust': 54000, 'min_thrust': 41000,
        'max_yawrate': 15, 'min_yawrate': -15,
        'hover': 43000
    }

    # Create instances for each drone
    drone1_pid_controller = DronePIDController('cf1', pid_params_cf1)
    drone2_pid_controller = DronePIDController('cf2', pid_params_cf2)

    # Spin both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(drone1_pid_controller)
    executor.add_node(drone2_pid_controller)

    try:
        executor.spin()
    finally:
        drone1_pid_controller.destroy_node()
        drone2_pid_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()