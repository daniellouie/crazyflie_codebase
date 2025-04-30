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
    pubNode = rclpy.create_node('viz_pub_node')
    pub_cur_cluster = pubNode.create_publisher(PoseStamped, '/cluster_state', 10)
    pub_des_cluster = pubNode.create_publisher(PoseStamped, '/cluster_desired', 10)
    
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
    des_cluster_positions = []
    des_cf1_positions = []
    des_cf2_positions = []

    try:
        while rclpy.ok():
            # Allow ROS to process messages
            # TODO : adjust the timeout as needed (too short and it will miss messages, too long and it will slow down the loop)
            executor.spin_once(timeout_sec=0.01)
            # Update cluster with position data from OptiTrack

            # needs to be assigned to temp variables to avoid race condition errors
            cur_position_cf1 = optitrack_subscriber_cf1.get_position()  # Drone 1 position
            cur_position_cf2 = optitrack_subscriber_cf2.get_position()  # Drone 2 position
            
            # print("cf1 position:", cur_position_cf1)
            # print("cf2 position:", cur_position_cf2)

            # Update cluster with the current positions of the drones
            cluster.updatePositions(cur_position_cf1, cur_position_cf2)

            # Perform cluster calculations
            cluster.update()

            # print("cf1 commanded position:", cluster.R_cmd[0:3])
            # print("cf2 commanded position:", cluster.R_cmd[4:7])


            # Publish messages for visualization on RViz2
            cur_cluster_msg = PoseStamped()
            cur_cluster_msg.header.stamp = pubNode.get_clock().now().to_msg()
            cur_cluster_msg.header.frame_id = "world"
            #need to convert to Anne's frame for visualization
            # NOTE : transformations are weird to show up correctly in Rviz
            cur_cluster_our_frame = cluster.frameAnneToOurs(cluster.C_cur)
            cur_cluster_msg.pose.position.x = cur_cluster_our_frame[0]
            cur_cluster_msg.pose.position.y = cur_cluster_our_frame[1]
            cur_cluster_msg.pose.position.z = cur_cluster_our_frame[2]
            pub_cur_cluster.publish(cur_cluster_msg)

            des_cluster_msg = PoseStamped()
            des_cluster_msg.header.stamp = pubNode.get_clock().now().to_msg()
            des_cluster_msg.header.frame_id = "world"
            #need to convert to Anne's frame for visualization
            des_cluster_our_frame = cluster.frameAnneToOurs(cluster.C_des)
            des_cluster_msg.pose.position.x = des_cluster_our_frame[0]
            des_cluster_msg.pose.position.y = des_cluster_our_frame[1]
            des_cluster_msg.pose.position.z = des_cluster_our_frame[2]
            pub_des_cluster.publish(des_cluster_msg)

            # publish R_cmd values for each drone for PID control
            des_cf1_msg = PoseStamped()
            des_cf1_msg.header.stamp = pubNode.get_clock().now().to_msg()
            des_cf1_msg.header.frame_id = "world"
            #R_cmd is already in our frame
            des_cf1_msg.pose.position.x = cluster.R_cmd[0]
            des_cf1_msg.pose.position.y = cluster.R_cmd[1]
            des_cf1_msg.pose.position.z = cluster.R_cmd[2]
            pub_cf1_cmd.publish(des_cf1_msg)

            des_cf2_msg = PoseStamped()
            des_cf2_msg.header.stamp = pubNode.get_clock().now().to_msg()
            des_cf2_msg.header.frame_id = "world"
            #R_cmd is already in our frame
            des_cf2_msg.pose.position.x = cluster.R_cmd[4]
            des_cf2_msg.pose.position.y = cluster.R_cmd[5]
            des_cf2_msg.pose.position.z = cluster.R_cmd[6]
            pub_cf2_cmd.publish(des_cf2_msg)

            # Record timestamp and positions
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            timestamps.append(current_time)
            cur_cf1_positions.append(cur_position_cf1)
            cur_cf2_positions.append(cur_position_cf2)
            cur_cluster_positions.append(cur_cluster_our_frame[:3])
            des_cluster_positions.append(des_cluster_our_frame[:3])
            des_cf1_positions.append(cluster.R_cmd[:3])
            des_cf2_positions.append(cluster.R_cmd[4:7])



    except KeyboardInterrupt:
        print("Shutting down...")
        #save flight data
        # Save data to CSV
        with open(f'cluster_data/cluster_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Cur_CF1_X', 'Cur_CF1_Y', 'Cur_CF1_Z',
                            'Cur_CF2_X', 'Cur_CF2_Y', 'Cur_CF2_Z',
                            'Cur_Cluster_X', 'Cur_Cluster_Y', 'Cur_Cluster_Z',
                            'Des_Cluster_X', 'Des_Cluster_Y', 'Des_Cluster_Z',
                            'Des_CF1_X', 'Des_CF1_Y', 'Des_CF1_Z',
                            'Des_CF2_X', 'Des_CF2_Y', 'Des_CF2_Z'])
            for i in range(len(timestamps)):
                writer.writerow([
                    timestamps[i],
                    *cur_cf1_positions[i],
                    *cur_cf2_positions[i],
                    *cur_cluster_positions[i],
                    *des_cluster_positions[i],
                    *des_cf1_positions[i],
                    *des_cf2_positions[i]
                ])

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
    finally:
        # Shutdown ROS nodes
        optitrack_subscriber_cf1.destroy_node()
        optitrack_subscriber_cf2.destroy_node()
        rclpy.shutdown()

        


if __name__ == '__main__':
    print("Starting Cluster Controller...")
    main()