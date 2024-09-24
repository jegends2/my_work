import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_path = None
        self.current_index = 0
        self.create_timer(0.1, self.update_path)

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Laser scan data
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )
        self.laser_data = []

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0

    def laser_scan_callback(self, msg):
        self.laser_data = msg.ranges

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def distance_to_goal(self, goal_x, goal_y):
        return math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)

    def angle_to_goal(self, goal_x, goal_y):
        return math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)

    def update_path(self):
        if self.current_path is None or not self.current_path.poses:
            return

        if self.current_index >= len(self.current_path.poses):
            self.stop_robot()
            return

        goal_pose = self.current_path.poses[self.current_index]
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        distance = self.distance_to_goal(goal_x, goal_y)
        angle_to_goal = self.angle_to_goal(goal_x, goal_y)
        angle_error = angle_to_goal - self.robot_theta

        cmd = Twist()
        if distance > 0.1:
            # Check for obstacles
            if self.check_for_obstacles():
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Rotate in place if obstacle detected
            else:
                cmd.linear.x = min(0.5, distance)
                cmd.angular.z = 2.0 * angle_error
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.current_index += 1

        self.publisher.publish(cmd)
        self.get_logger().info(f'Publishing cmd_vel: linear_x={cmd.linear.x}, angular_z={cmd.angular.z}')

    def check_for_obstacles(self):
        # Simple obstacle check based on laser scan data
        # Assuming laser_data is in polar coordinates
        threshold_distance = 1.0  # meters
        for distance in self.laser_data:
            if distance < threshold_distance:
                return True
        return False

    def stop_robot(self):
        cmd = Twist()
        se
