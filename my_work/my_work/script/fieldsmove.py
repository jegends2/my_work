import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.publisher_ = self.create_publisher(Path, '/move_base/GlobalPlanner/plan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.load_path()

    def load_path(self):
        """Load path from CSV and publish as ROS2 Path message."""
        path = Path()
        path.header.frame_id = "map"
        with open('path.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                x, y = map(float, row)
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                path.poses.append(pose)
        self.publisher_.publish(path)

    def timer_callback(self):
        self.get_logger().info('Publishing path...')

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
