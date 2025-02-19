import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import threading
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.command_publisher = self.create_publisher(Int32, '/vel_pub', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.safe = True  
        self.last_command = 5  
        
        # Fake odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel = 0.01  
        self.angular_vel = 0.01
        
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
       
        self.odom_timer = self.create_timer(0.1, self.update_odometry)

        self.user_input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.user_input_thread.start()

    def command_loop(self):
        
        while rclpy.ok():
            self.publish_command()

    def publish_command(self):
        
        try:
            mess = Int32()
            if self.safe:
                user_input = input("Enter 8 (Forward), 2 (Backward), 4 (Left), 6 (Right), 5 (Stop): ")
                mess.data = int(user_input)
                self.last_command = mess.data  

            self.command_publisher.publish(mess)
            self.get_logger().info(f'Publishing: {mess.data}')
        except ValueError:
            self.get_logger().error("Invalid input! Please enter an integer.")

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
            self.last_command = 5  
            self.get_logger().warn(f" Obstacle detected! Robot stopped.\n")

        elif not obstacle_detected and not self.safe:
            print(" Path is clear. Press Enter to resume control... ")
            self.safe = True

    def update_odometry(self):
        """ Updates fake odometry data based on movement commands """
        if self.last_command == 8: 
            self.vel = 0.01
            self.x += self.vel * math.cos(self.theta)
            self.y += self.vel * math.sin(self.theta)
        elif self.last_command == 2:  
            self.vel = 0.01
            self.x -= self.vel * math.cos(self.theta)
            self.y -= self.vel * math.sin(self.theta)
        elif self.last_command == 4:  
            self.vel = 0
            self.theta += self.angular_vel
        elif self.last_command == 6:  
            self.vel = 0
            self.theta -= self.angular_vel

        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation = q
        
        self.odom_publisher.publish(odom_msg)

        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)
        
        self.get_logger().info(f' Odom: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')


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
