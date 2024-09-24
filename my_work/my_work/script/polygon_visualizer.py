import json
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class PolygonVisualizer(Node):
    def __init__(self):
        super().__init__('polygon_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Waiting for timer callback...')
        
        # Load and publish markers
        self.load_and_publish_markers()

    def load_and_publish_markers(self):
        with open('headland_polygon.json', 'r') as f:
            data = json.load(f)
        
        polygons = data['polygons']
        
        if polygons:
            # Assuming we are visualizing the first polygon
            coords = polygons[0]
            
            # Create a marker for the polygon
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'polygon'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Line width
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
            
            for coord in coords:
                point = Point()
                point.x = float(coord[0])  # Ensure float type
                point.y = float(coord[1])  # Ensure float type
                point.z = 0.0  # Explicitly set z to float
                marker.points.append(point)
            
            # Close the polygon by adding the first point at the end
            if len(marker.points) > 0:
                marker.points.append(marker.points[0])
            
            # Publish Marker
            self.publisher_.publish(marker)

    def timer_callback(self):
        self.get_logger().info('Publishing markers')

def main(args=None):
    rclpy.init(args=args)
    visualizer = PolygonVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
