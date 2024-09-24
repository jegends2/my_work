import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # Parameters
        self.desired_distance = 0.5  # Desired distance from the wall in meters
        self.min_distance = 0.4  # Minimum allowed distance to the wall
        self.max_distance = 0.6  # Maximum allowed distance to the wall
        self.forward_speed = 0.2
        self.angular_speed = 0.3  # Adjust angular speed for smoother turns

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.laser_ranges = []

    def laser_callback(self, msg):
        # Store laser scan data
        self.laser_ranges = msg.ranges

    def timer_callback(self):
        if not self.laser_ranges:
            return

        # Assuming left wall detection
        left_ranges = self.laser_ranges[30:60]
        if len(left_ranges) == 0:
            return

        # Find the minimum distance to the wall on the left side
        left_range = min(left_ranges)
        
        twist = Twist()

        # Check distance and adjust speed and direction
        if left_range < self.min_distance:
            # Too close to the wall, stop and rotate right
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
        elif left_range > self.max_distance:
            # Too far from the wall, move forward and rotate left
            twist.linear.x = self.forward_speed
            twist.angular.z = self.angular_speed
        else:
            # Distance is within the acceptable range, move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        # Publish command
        self.cmd_vel_pub.publish(twist)

        # Log for debugging
        self.get_logger().info(f"Distance to wall: {left_range:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
