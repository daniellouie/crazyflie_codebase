import rclpy
from rclpy.executors import MultiThreadedExecutor #two diff approaches for spinning multiple nodes
from rclpy.executors import SingleThreadedExecutor
from .cluster import Cluster
from .clusterOptitrackSubscriber import ClusterOptitrackSubscriber
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import os



def main(args=None):
    rclpy.init(args=args)

    # Initialize OptiTrack subscribers for two drones
    optitrack_subscriber_cf1 = ClusterOptitrackSubscriber('cf1')
    optitrack_subscriber_cf2 = ClusterOptitrackSubscriber('cf2')

    # Initialize the Cluster class
    cluster = Cluster()

    # set desired cluster position in our frame(x, y, z, alpha, beta, phi1, phi2, p)
    cluster.C_des = cluster.frameOursToAnne([1.5, 1, 1, 0, 0, 0, 0, 1]) # convert to Anne's frame 

    # publish current cluster Pose for visualization
    # TODO : need to figure out the issue of 3 duplicate nodes
    pubNode = rclpy.create_node('cluster_control_pub_node')

    # publishers for R_cmd values for each drone for PID control
    pub_cf1_cmd = pubNode.create_publisher(PoseStamped, '/cf1_cmd', 10)
    pub_cf2_cmd = pubNode.create_publisher(PoseStamped, '/cf2_cmd', 10)
    
    # Create a multi-threaded executor
    executor = MultiThreadedExecutor()

    # Add the OptiTrackSubscriber nodes to the executor
    # allows both subscribers to run concurrently
    executor.add_node(optitrack_subscriber_cf1)
    executor.add_node(optitrack_subscriber_cf2)
    executor.add_node(pubNode)

    # Data storage for recording positions
    timestamps = []
    cur_cf1_positions = []
    cur_cf2_positions = []
    cur_cluster_positions = []
    cur_cluster_variables = []
    des_cluster_positions = []
    des_cluster_variables = []
    des_cf1_positions = []
    safe_cf1_positions = []
    des_cf2_positions = []
    safe_cf2_positions = []

    # Frame capture interval (how often data should be collected for graphing)
    FRAME_CAPTURE_INTERVAL = 0.1 # frequency in seconds
    last_capture_time = datetime.now()

    try:
        while rclpy.ok():
            # Allow ROS to process messages
            # TODO : adjust the timeout as needed (too short and it will miss messages, too long and it will slow down the loop)
            executor.spin_once(timeout_sec=0.01)
            # Update cluster with position data from OptiTrack

            # needs to be assigned to temp variables to avoid race condition errors
            cur_position_cf1 = optitrack_subscriber_cf1.get_position()  # Drone 1 position
            cur_position_cf2 = optitrack_subscriber_cf2.get_position()  # Drone 2 position

            # Update cluster with the current positions of the drones
            cluster.updatePositions(cur_position_cf1, cur_position_cf2)

            # Perform cluster calculations
            cluster.update()

            
            # publish R_cmd values for each drone for PID control
            des_cf1_msg = PoseStamped()
            des_cf1_msg.header.stamp = pubNode.get_clock().now().to_msg()
            des_cf1_msg.header.frame_id = "world"
            #R_cmd is already in our frame
            original_des_cf1 = cluster.R_cmd[:3]
            # clamp the values betweeen 0 and 3 for safety (prevents commands outside of test space)
            safe_cf1_cmd = [
                max(0, min(original_des_cf1[0], 3)),  # x clamp cf 1
                max(0, min(original_des_cf1[1], 3)),  # y clamp cf 1
                max(0, min(original_des_cf1[2], 3))   # z clamp cf 1
            ]
            des_cf1_msg.pose.position.x = safe_cf1_cmd[0]
            des_cf1_msg.pose.position.y = safe_cf1_cmd[1]
            des_cf1_msg.pose.position.z = safe_cf1_cmd[2]
            pub_cf1_cmd.publish(des_cf1_msg)

            des_cf2_msg = PoseStamped()
            des_cf2_msg.header.stamp = pubNode.get_clock().now().to_msg()
            des_cf2_msg.header.frame_id = "world"
            #R_cmd is already in our frame
            original_des_cf2 = cluster.R_cmd[4:7]
            # clamp the values betweeen 0 and 3 for safety (prevents commands outside of test space)
            safe_cf2_cmd = [
                max(0, min(original_des_cf2[0], 3)),  # x clamp cf 2
                max(0, min(original_des_cf2[1], 3)),  # y clamp cf 2
                max(0, min(original_des_cf2[2], 3))   # z clamp cf 2
            ]
            des_cf2_msg.pose.position.x = safe_cf2_cmd[0]
            des_cf2_msg.pose.position.y = safe_cf2_cmd[1]
            des_cf2_msg.pose.position.z = safe_cf2_cmd[2]
            pub_cf2_cmd.publish(des_cf2_msg)

            # conver current and desired cluster positions to our frame
            cur_cluster_our_frame = cluster.frameAnneToOurs(cluster.C_cur)  # needed for data collection
            des_cluster_our_frame = cluster.frameAnneToOurs(cluster.C_des)  # needed for data collection

            # check if enough time has passed to capture a frame
            # if so, record the current positions and timestamp
            current_time = datetime.now()
            if (current_time - last_capture_time).total_seconds() >= FRAME_CAPTURE_INTERVAL:
                # Record timestamp and positions
                timestamps.append(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))
                cur_cf1_positions.append(cur_position_cf1)
                cur_cf2_positions.append(cur_position_cf2)
                cur_cluster_positions.append(cur_cluster_our_frame[:3])
                des_cluster_positions.append(des_cluster_our_frame[:3])
                des_cf1_positions.append(cluster.R_cmd[:3])
                safe_cf1_positions.append(safe_cf1_cmd)
                des_cf2_positions.append(cluster.R_cmd[4:7])
                safe_cf2_positions.append(safe_cf2_cmd)
                #extract the cluster variables
                cur_alpha, cur_beta, cur_phi1, cur_phi2, cur_p = cluster.C_cur[3:8]
                des_alpha, des_beta, des_phi1, des_phi2, des_p = cluster.C_des[3:8]
                cur_cluster_variables.append([cur_alpha, cur_beta, cur_phi1, cur_phi2, cur_p])
                des_cluster_variables.append([des_alpha, des_beta, des_phi1, des_phi2, des_p])
                #update last capture time
                last_capture_time = current_time


    except KeyboardInterrupt:
        print("Shutting down...")
        #save flight data

        #ensure directory exists
        os.makedirs('cluster_data', exist_ok=True)
        # Save data to CSV
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
            for i in range(len(timestamps)):
                writer.writerow([
                    timestamps[i],
                    *cur_cf1_positions[i],
                    *cur_cf2_positions[i],
                    *cur_cluster_positions[i],
                    *cur_cluster_variables[i],
                    *des_cluster_positions[i],
                    *des_cluster_variables[i],
                    *des_cf1_positions[i],
                    *safe_cf1_positions[i],
                    *des_cf2_positions[i],
                    *safe_cf2_positions[i]
                ])

    finally:
        # Shutdown ROS nodes
        optitrack_subscriber_cf1.destroy_node()
        optitrack_subscriber_cf2.destroy_node()
        rclpy.shutdown()

        # Plot data
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Convert lists to numpy arrays for easier slicing
        cur_cf1_positions = np.array(cur_cf1_positions)
        cur_cf2_positions = np.array(cur_cf2_positions)
        cur_cluster_positions = np.array(cur_cluster_positions)
        des_cluster_positions = np.array(des_cluster_positions)
        des_cf1_positions = np.array(des_cf1_positions)
        des_cf2_positions = np.array(des_cf2_positions)

        # Plot each dataset
        ax.scatter(cur_cf1_positions[:, 0], cur_cf1_positions[:, 1], cur_cf1_positions[:, 2], label='Cur_CF1', c='r')
        ax.scatter(cur_cf2_positions[:, 0], cur_cf2_positions[:, 1], cur_cf2_positions[:, 2], label='Cur_CF2', c='g')
        ax.scatter(cur_cluster_positions[:, 0], cur_cluster_positions[:, 1], cur_cluster_positions[:, 2], label='Cur_Cluster', c='b')
        ax.scatter(des_cluster_positions[:, 0], des_cluster_positions[:, 1], des_cluster_positions[:, 2], label='Des_Cluster', c='y')
        ax.scatter(des_cf1_positions[:, 0], des_cf1_positions[:, 1], des_cf1_positions[:, 2], label='Des_CF1', c='m')
        ax.scatter(des_cf2_positions[:, 0], des_cf2_positions[:, 1], des_cf2_positions[:, 2], label='Des_CF2', c='c')

        # Label axes
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.legend()
        plt.title('3D Position Data')
        plt.show()
        print("plotting 3D data")


if __name__ == '__main__':
    print("Starting Cluster Controller...")
    main()