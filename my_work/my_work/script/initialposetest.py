import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)


    def timer_callback(self):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        # Example coordinates, adjust as needed
        pose.pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.pose.orientation.x = msg.pose.pose.orientation
#        pose.pose.pose.orientation.y = 0.0
#        pose.pose.pose.orientation.z = 0.0
#        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance = [0.0] * 36
        self.publisher_.publish(pose)
        self.get_logger().info(f'Publishing initial pose: ({pose.pose.pose.position.x}, {pose.pose.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

