import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowing(Node):
    def __init__(self):
        super().__init__('wall_following')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        self.target_distance = 0.5  # Desired distance from the wall in meters
        self.forward_speed = 0.1
        self.angular_speed = 0.3

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.laser_ranges = []

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges

    def timer_callback(self):
        if not self.laser_ranges:
            return

        front_distance = min(min(self.laser_ranges[0:30]), min(self.laser_ranges[-30:]))
        left_distance = min(self.laser_ranges[60:120])

        twist = Twist()

        if front_distance < self.target_distance:
            # If there's an obstacle in front, rotate right
            twist.angular.z = -self.angular_speed
        elif left_distance > self.target_distance:
            # If the wall is too far on the left, move slightly left
            twist.angular.z = self.angular_speed
            twist.linear.x = self.forward_speed
        elif left_distance < self.target_distance:
            # If the wall is too close on the left, move slightly right
            twist.angular.z = -self.angular_speed
            twist.linear.x = self.forward_speed
        else:
            # Move forward if no correction is needed
            twist.linear.x = self.forward_speed

        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f"Front: {front_distance:.2f} m, Left: {left_distance:.2f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
