import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class VisualSender(Node):
    def __init__(self):
        super().__init__('visual_sender')

        # Publisher for visualizing the goal in RViz as a marker
        self.visualizer_pub = self.create_publisher(Marker, 'visualization_marker', QoSProfile(depth=10))
        # Publisher for sending the goal to the navigation system
        self.goal_pub = self.create_publisher(PoseStamped, 'goal', QoSProfile(depth=10))

        # Input handling loop
        self.handle_input()

    def handle_input(self):
        while True:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            
            self.visualize_goal(x, y)
            
            command = input("Enter 'yes' to move to the goal or 'no' to input a new coordinate: ").strip().lower()
            if command == 'yes':
                self.send_goal(x, y)
                break
            elif command == 'no':
                continue
            else:
                print("Invalid command. Please enter 'yes' or 'no'.")

    def visualize_goal(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goals'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Fixed z-coordinate for 2D visualization
        marker.pose.orientation.w = 1.0  # Identity orientation
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.visualizer_pub.publish(marker)
        self.get_logger().info(f"Visualizing goal at x={x}, y={y}")

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0  # Fixed z-coordinate for 2D navigation
        msg.pose.orientation.w = 1.0  # Identity orientation

        self.goal_pub.publish(msg)
        self.get_logger().info(f"Sending goal to x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
