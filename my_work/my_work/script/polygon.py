import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(
            Path,
            '/path',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_path = None
        self.current_index = 0
        self.create_timer(0.1, self.update_path)

        # Initial robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.subscription_odometry = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def listener_callback(self, msg):
        self.current_path = msg
        self.current_index = 0  # Start from the beginning of the path

    def odom_callback(self, msg):
        # Update robot position from odometry
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
            cmd.linear.x = min(0.5, distance)
            cmd.angular.z = 2.0 * angle_error
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.current_index += 1

        self.publisher.publish(cmd)
        self.get_logger().info(f'Publishing cmd_vel: linear_x={cmd.linear.x}, angular_z={cmd.angular.z}')

    def stop_robot(self):
        cmd = Twist()
        self.publisher.publish(cmd)
        self.get_logger().info('Path completed, stopping robot.')

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
