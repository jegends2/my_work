import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.publisher = self.create_publisher(Path, '/navigate_to_path', 10)
        self.path = Path()
        # Set up your path here
        self.path.header.frame_id = 'map'
        # Add path points
        self.publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
