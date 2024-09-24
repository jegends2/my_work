import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CoordinateVisualizer(Node):
    def __init__(self):
        super().__init__('coordinate_visualizer')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.get_logger().info("Enter coordinates to visualize.")
        self.get_coordinates()

    def get_coordinates(self):
        try:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            self.visualize_coordinates(x, y)
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
            self.get_coordinates()

    def visualize_coordinates(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'coordinate_visualizer'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Fixed z-coordinate
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=5).to_msg()

        self.marker_publisher.publish(marker)
        self.get_logger().info(f'Published marker at x={x}, y={y}')
        # Continue waiting for new coordinates
        self.get_coordinates()

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
