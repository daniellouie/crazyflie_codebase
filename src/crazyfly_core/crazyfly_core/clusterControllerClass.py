import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .cluster import Cluster
from .clusterOptitrackSubscriber import ClusterOptitrackSubscriber
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import csv
import os



class ClusterController(Node):
    def __init__(self):
        super().__init__('cluster_controller')

        # Initialize OptiTrack subscribers
        self.optitrack_cf1 = ClusterOptitrackSubscriber('cf1')
        self.optitrack_cf2 = ClusterOptitrackSubscriber('cf2')

        # Initialize the Cluster class
        self.cluster = Cluster()
        self.cluster.C_des = self.cluster.frameOursToAnne([1.5, 1, 1, 0, 0, 0, 0, 1])

        # Publishers
        self.pub_cf1_cmd = self.create_publisher(PoseStamped, '/cf1_cmd', 10)
        self.pub_cf2_cmd = self.create_publisher(PoseStamped, '/cf2_cmd', 10)

        # Data storage for recording positions
        self.timestamps = []
        self.cur_cf1_positions = []
        self.cur_cf2_positions = []
        self.cur_cluster_positions = []
        self.cur_cluster_variables = []
        self.des_cluster_positions = []
        self.des_cluster_variables = []
        self.des_cf1_positions = []
        self.safe_cf1_positions = []
        self.des_cf2_positions = []
        self.safe_cf2_positions = []

        # Frame capture interval (how often data should be collected for graphing)
        self.frame_capture_interval = 0.1  # frequency in seconds
        self.last_capture_time = datetime.now()

    def update_positions(self):
        # Get current positions from OptiTrack
        cur_position_cf1 = self.optitrack_cf1.get_position()
        cur_position_cf2 = self.optitrack_cf2.get_position()

        # Update cluster with the current positions of the drones
        self.cluster.updatePositions(cur_position_cf1, cur_position_cf2)

    def perform_cluster_calculations(self):
        # Perform cluster calculations
        self.cluster.update()

    def publish_commands(self):
        # Publish R_cmd values for each drone for PID control
        des_cf1_msg = PoseStamped()
        des_cf1_msg.header.stamp = self.get_clock().now().to_msg()
        des_cf1_msg.header.frame_id = "world"

        # Clamp the values between 0 and 3 for safety
        original_des_cf1 = self.cluster.R_cmd[:3]
        safe_cf1_cmd = [
            max(0, min(original_des_cf1[0], 3)),
            max(0, min(original_des_cf1[1], 3)),
            max(0, min(original_des_cf1[2], 3))
        ]
        des_cf1_msg.pose.position.x = safe_cf1_cmd[0]
        des_cf1_msg.pose.position.y = safe_cf1_cmd[1]
        des_cf1_msg.pose.position.z = safe_cf1_cmd[2]
        self.pub_cf1_cmd.publish(des_cf1_msg)

        des_cf2_msg = PoseStamped()
        des_cf2_msg.header.stamp = self.get_clock().now().to_msg()
        des_cf2_msg.header.frame_id = "world"

        original_des_cf2 = self.cluster.R_cmd[4:7]
        safe_cf2_cmd = [
            max(0, min(original_des_cf2[0], 3)),
            max(0, min(original_des_cf2[1], 3)),
            max(0, min(original_des_cf2[2], 3))
        ]
        des_cf2_msg.pose.position.x = safe_cf2_cmd[0]
        des_cf2_msg.pose.position.y = safe_cf2_cmd[1]
        des_cf2_msg.pose.position.z = safe_cf2_cmd[2]
        self.pub_cf2_cmd.publish(des_cf2_msg)

        # Store safe commands for logging
        self.safe_cf1_positions.append(safe_cf1_cmd)
        self.safe_cf2_positions.append(safe_cf2_cmd)

    def record_data(self):
        # Record current and desired positions for graphing
        current_time = datetime.now()
        if (current_time - self.last_capture_time).total_seconds() >= self.frame_capture_interval:
            self.timestamps.append(current_time.strftime('%Y-%m-%d %H:%M:%S.%f'))

            cur_position_cf1 = self.optitrack_cf1.get_position()
            cur_position_cf2 = self.optitrack_cf2.get_position()

            cur_cluster_our_frame = self.cluster.frameAnneToOurs(self.cluster.C_cur)
            des_cluster_our_frame = self.cluster.frameAnneToOurs(self.cluster.C_des)

            self.cur_cf1_positions.append(cur_position_cf1)
            self.cur_cf2_positions.append(cur_position_cf2)
            self.cur_cluster_positions.append(cur_cluster_our_frame[:3])
            self.des_cluster_positions.append(des_cluster_our_frame[:3])
            self.des_cf1_positions.append(self.cluster.R_cmd[:3])
            self.des_cf2_positions.append(self.cluster.R_cmd[4:7])

            cur_alpha, cur_beta, cur_phi1, cur_phi2, cur_p = self.cluster.C_cur[3:8]
            des_alpha, des_beta, des_phi1, des_phi2, des_p = self.cluster.C_des[3:8]
            self.cur_cluster_variables.append([cur_alpha, cur_beta, cur_phi1, cur_phi2, cur_p])
            self.des_cluster_variables.append([des_alpha, des_beta, des_phi1, des_phi2, des_p])

            self.last_capture_time = current_time

    def save_data_to_csv(self):
        # Save recorded data to a CSV file
        os.makedirs('cluster_data', exist_ok=True)
        with open(f'cluster_data/cluster_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp',
                             'Cur_CF1_X', 'Cur_CF1_Y', 'Cur_CF1_Z',
                             'Cur_CF2_X', 'Cur_CF2_Y', 'Cur_CF2_Z',
                             'Cur_Cluster_X', 'Cur_Cluster_Y', 'Cur_Cluster_Z',
                             'Cur_Alpha', 'Cur_Beta', 'Cur_Phi1', 'Cur_Phi2', 'Cur_P',
                             'Des_Cluster_X', 'Des_Cluster_Y', 'Des_Cluster_Z',
                             'Des_Alpha', 'Des_Beta', 'Des_Phi1', 'Des_Phi2', 'Des_P',
                             'Des_CF1_X', 'Des_CF1_Y', 'Des_CF1_Z',
                             'Safe_CF1_X', 'Safe_CF1_Y', 'Safe_CF1_Z',
                             'Des_CF2_X', 'Des_CF2_Y', 'Des_CF2_Z',
                             'Safe_CF2_X', 'Safe_CF2_Y', 'Safe_CF2_Z'])
            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i],
                    *self.cur_cf1_positions[i],
                    *self.cur_cf2_positions[i],
                    *self.cur_cluster_positions[i],
                    *self.cur_cluster_variables[i],
                    *self.des_cluster_positions[i],
                    *self.des_cluster_variables[i],
                    *self.des_cf1_positions[i],
                    *self.safe_cf1_positions[i],
                    *self.des_cf2_positions[i],
                    *self.safe_cf2_positions[i]
                ])

    def plot_data(self):
        # Plot 3D position data
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        cur_cf1_positions = np.array(self.cur_cf1_positions)
        cur_cf2_positions = np.array(self.cur_cf2_positions)
        cur_cluster_positions = np.array(self.cur_cluster_positions)
        des_cluster_positions = np.array(self.des_cluster_positions)
        des_cf1_positions = np.array(self.des_cf1_positions)
        des_cf2_positions = np.array(self.des_cf2_positions)

        ax.scatter(cur_cf1_positions[:, 0], cur_cf1_positions[:, 1], cur_cf1_positions[:, 2], label='Cur_CF1', c='r')
        ax.scatter(cur_cf2_positions[:, 0], cur_cf2_positions[:, 1], cur_cf2_positions[:, 2], label='Cur_CF2', c='g')
        ax.scatter(cur_cluster_positions[:, 0], cur_cluster_positions[:, 1], cur_cluster_positions[:, 2], label='Cur_Cluster', c='b')
        ax.scatter(des_cluster_positions[:, 0], des_cluster_positions[:, 1], des_cluster_positions[:, 2], label='Des_Cluster', c='y')
        ax.scatter(des_cf1_positions[:, 0], des_cf1_positions[:, 1], des_cf1_positions[:, 2], label='Des_CF1', c='m')
        ax.scatter(des_cf2_positions[:, 0], des_cf2_positions[:, 1], des_cf2_positions[:, 2], label='Des_CF2', c='c')

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.legend()
        plt.title('3D Position Data')
        plt.show()

    def run(self):
        # Use MultiThreadedExecutor to handle concurrency
        executor = MultiThreadedExecutor()
        executor.add_node(self.optitrack_cf1)
        executor.add_node(self.optitrack_cf2)
        executor.add_node(self)

        try:
            while rclpy.ok(): 
                executor.spin_once(timeout_sec=0.01)
                self.update_positions()
                self.perform_cluster_calculations()
                self.publish_commands()
                self.record_data()
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
           
        finally:
            self.save_data_to_csv()
            executor.shutdown()
            self.destroy_node()
            rclpy.shutdown()
            

def main(args=None):
    rclpy.init(args=args)
    controller = ClusterController()
    controller.run()



if __name__ == '__main__':
    main()