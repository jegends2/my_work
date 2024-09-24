import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotStopper(Node):
    def __init__(self):
        super().__init__('robot_stopper')
        self.subscription = self.create_subscription(
            String,
            'stop_command',
            self.stop_command_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Robot Stopper Node has been started.")

    def stop_command_callback(self, msg):
        if msg.data == 'stop':
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("Robot has been stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotStopper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
