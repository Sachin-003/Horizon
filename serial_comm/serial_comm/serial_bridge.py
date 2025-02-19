import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.subscription = self.create_subscription(Int32, 'vel_pub', self.vel_sub_callback, 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=5)  
            self.get_logger().info('Serial connection established.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')

    def vel_sub_callback(self, msg):
        """Send received velocity command to Arduino."""
        self.get_logger().info(f'Sending to Arduino: {msg.data}')
        self.serial_port.write((str(msg.data) + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
