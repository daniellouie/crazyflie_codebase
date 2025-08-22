import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import time
from pathlib import Path


# collects position data for a stationary object and saves that data to a csv, should be compared with real life position data
# has not been tested yet
class PoseLogger(Node):
    def __init__(self, duration=30.0):
        super().__init__('pose_logger')
        self.duration = duration
        self.start_time = self.get_clock().now()
        self.subscriber = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf2/pose',
            self.pose_callback,
            10
        )
        self.pose_data = []
        self.get_logger().info(f"Logging data for {self.duration} seconds...")

    def pose_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if elapsed > self.duration:
            self.get_logger().info("Finished recording. Saving to CSV...")
            self.save_to_csv()
            rclpy.shutdown()
            return

        position = msg.pose.position
        self.pose_data.append([elapsed, position.x, position.y, position.z])
    
    def save_to_csv(self):
        out_file = Path.home() / 'cf2_pose_log.csv'
        with open(out_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time (s)', 'X', 'Y', 'Z'])
            writer.writerows(self.pose_data)
        self.get_logger().info(f"Saved CSV to: {out_file}")

def main(args=None):
    rclpy.init(args=args)
    duration_seconds = 30  # set your duration here
    node = PoseLogger(duration=duration_seconds)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
