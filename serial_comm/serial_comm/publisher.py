import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match YDLidar's QoS
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.command_publisher = self.create_publisher(Int32, '/vel_pub', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.safe = True  

        
        self.user_input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.user_input_thread.start()

    def command_loop(self):
        """ Continuously runs in a separate thread to accept user input """
        while rclpy.ok():
            self.publish_command()

    def publish_command(self):
        """ Handles user input and publishes movement commands """
        try:
            mess = Int32()

            # if not self.safe:
            #     self.get_logger().warn("🚨 Obstacle detected! Ignoring input and stopping robot.")
            #     mess.data = 5  
            if self.safe:
                user_input = input("Enter 8 (Forward), 2 (Backward), 4 (Left), 6 (Right), 5 (Stop): ")
                mess.data = int(user_input)

            self.command_publisher.publish(mess)
            self.get_logger().info(f'📡 Publishing: {mess.data}')
        except ValueError:
            self.get_logger().error("❌ Invalid input! Please enter an integer.")

    def lidar_callback(self, msg):
        """ Processes LiDAR data to detect obstacles """
    
        range_arr = msg.ranges
        front_arr = range_arr[-30:] + range_arr[:30]

        obstacle_detected = any(0.1 < range <= 0.5 for range in front_arr)

        if obstacle_detected and self.safe:
            stop_cmd = Int32()
            stop_cmd.data = 5
            self.command_publisher.publish(stop_cmd)
            self.safe = False
            self.get_logger().warn(f"🚨 Obstacle detected! Robot stopped.\n")

        elif not obstacle_detected and not self.safe:
            print("✅ Path is clear. Press Enter to resume control... ")
            self.safe = True

        


def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
