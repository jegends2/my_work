import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from shapely.geometry import Polygon, LineString, MultiLineString, box
from shapely.ops import unary_union
import numpy as np

def create_polygon():
    # Define the polygon representing the map
    return Polygon([(-4.5, -3), (-4.5, 5), (1, 5), (1, 1), (9, 1), (9, -4), (7.5, -4), (7.5, -0.25), (7, 1), (-2.5, 1)])

def boustrophedon_decomposition(polygon, spacing=1.0):
    minx, miny, maxx, maxy = polygon.bounds
    lines = []
    x = minx + spacing
    while x <= maxx:
        line = LineString([(x, miny), (x, maxy)])
        if polygon.intersects(line):
            intersection = polygon.intersection(line)
            if isinstance(intersection, LineString):
                lines.append(intersection)
            elif isinstance(intersection, MultiLineString):
                for subline in intersection.geoms:
                    lines.append(subline)
        x += spacing
    return lines

def generate_coverage_path(polygon, spacing=1.0):
    # Generate boustrophedon decomposition lines
    lines = boustrophedon_decomposition(polygon, spacing)
    path = []

    for i, line in enumerate(lines):
        if i % 2 == 0:
            path.extend(list(line.coords))
        else:
            path.extend(list(line.coords)[::-1])
    return path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def generate_path(self):
        polygon = create_polygon()
        # Define padding to avoid walls, this value can be adjusted
        padding = 0.5
        # Inflate the polygon to avoid path generation too close to the boundaries
        inflated_polygon = polygon.buffer(-padding)
        path_coords = generate_coverage_path(inflated_polygon, spacing=0.5)
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for coord in path_coords:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = coord[0]
            pose.pose.position.y = coord[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        
        return ros_path

    def publish_path(self):
        path = self.generate_path()
        self.publisher_.publish(path)
        self.get_logger().info('Publishing path...')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
