import rclpy
from rclpy.executors import MultiThreadedExecutor
from cluster import Cluster
from optitrack_subscriber import OptiTrackSubscriber

def main(args=None):
    rclpy.init(args=args)

    # Initialize OptiTrack subscribers for two drones
    optitrack_subscriber_cf1 = OptiTrackSubscriber('cf1')
    optitrack_subscriber_cf2 = OptiTrackSubscriber('cf2')

    # Initialize the Cluster class
    cluster = Cluster()

    # Create a multi-threaded executor
    executor = MultiThreadedExecutor()

    # Add the OptiTrackSubscriber nodes to the executor
    # allows both subscribers to run concurrently
    executor.add_node(optitrack_subscriber_cf1)
    executor.add_node(optitrack_subscriber_cf2)

    try:
        while rclpy.ok():
            # Update cluster with position data from OptiTrack

            # RACE CONDITION, ensure the values sent to cluster are the latest and won't change mid calculation
            # assign to temp variables to send snapshot of the current positions
            cur_position_cf1 = optitrack_subscriber_cf1.get_position()  # Drone 1 position
            cur_position_cf2 = optitrack_subscriber_cf2.get_position()  # Drone 2 position

            # Update cluster with the current positions of the drones
            cluster.updatePositions(cur_position_cf1, cur_position_cf2)

            # Perform cluster calculations
            cluster.update()

            # Print cluster state for debugging
            print("Cluster state:", cluster.C_cur)

            # Allow ROS to process messages
            # TODO : adjust the timeout as needed (too short and it will miss messages, too long and it will slow down the loop)
            executor.spin_once(timeout_sec=0.01)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Shutdown ROS nodes
        optitrack_subscriber_cf1.destroy_node()
        optitrack_subscriber_cf2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()