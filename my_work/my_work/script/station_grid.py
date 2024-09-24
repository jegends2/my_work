import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import yaml
from PIL import Image

class MapGridVisualizer(Node):
    def __init__(self):
        super().__init__('map_grid_visualizer')

        # Declare and get parameters
        self.declare_parameter('map_file', '/home/adsol/box_map.yaml')
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value

        # Initialize parameters
        self.robot_size_x = 0.11  # Robot's x-axis size
        self.robot_size_y = 0.12  # Robot's y-axis size

        # Load the map
        self.load_map(self.map_file)

        # Create publishers for goal pose and markers
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.get_logger().info("Publishers created")

        # Create a timer to periodically update visualization
        self.timer = self.create_timer(1.0, self.update_visualization)  # 1 second interval

    def load_map(self, map_file):
        self.get_logger().info(f"Loading map from {map_file}")

        # Load the map from YAML file
        with open(map_file, 'r') as file:
            map_data = yaml.safe_load(file)

        self.resolution = map_data['resolution']
        self.origin = np.array(map_data['origin'])

        # Load the image file
        image_file = map_data['image']
        with Image.open(image_file) as img:
            self.width, self.height = img.size
            self.get_logger().info(f"Map dimensions: {self.width}x{self.height}")

            # Convert image to grayscale
            if img.mode != 'L':
                img = img.convert('L')
            self.data = np.array(img)

        # Convert grayscale image data to binary (1 for white, 0 for non-white)
        self.data = np.where(self.data >= 100, 1, 0)

        # Initialize the station position
        self.station_position = self.find_station_position()

    def find_station_position(self):
        # Convert the image coordinates to map coordinates
        min_x = int(self.origin[0] / self.resolution)
        min_y = int(self.origin[1] / self.resolution)
        max_x = min_x + int(self.robot_size_x / self.resolution)
        max_y = min_y + int(self.robot_size_y / self.resolution)

        # Ensure coordinates are within bounds
        min_x = max(0, min_x)
        min_y = max(0, min_y)
        max_x = min(self.width, max_x)
        max_y = min(self.height, max_y)

        # Check if the defined area is within bounds
        if 0 <= min_x < self.width and 0 <= min_y < self.height:
            white_region = self.data[min_y:max_y, min_x:max_x]
            if np.any(white_region == 1):
                # Set station position to the center of the defined area
                center_x = min_x + (max_x - min_x) / 2
                center_y = min_y + (max_y - min_y) / 2
                station_x = self.origin[0] + center_x * self.resolution
                station_y = self.origin[1] + center_y * self.resolution
                return (station_x, station_y)
            else:
                self.get_logger().warn("No white region found in the specified area.")
                return (self.origin[0], self.origin[1])
        else:
            self.get_logger().warn("Defined area is out of bounds.")
            return (self.origin[0], self.origin[1])

    def update_visualization(self):
        marker_array = MarkerArray()
        
        # Create a marker for the station area
        station_marker = Marker()
        station_marker.header.frame_id = 'map'
        station_marker.header.stamp = self.get_clock().now().to_msg()
        station_marker.ns = 'station'
        station_marker.id = 0
        station_marker.type = Marker.CUBE
        station_marker.action = Marker.ADD
        station_marker.pose.position.x = self.station_position[0]
        station_marker.pose.position.y = self.station_position[1]
        station_marker.pose.position.z = 0.0
        station_marker.pose.orientation.w = 1.0
        station_marker.scale.x = self.robot_size_x
        station_marker.scale.y = self.robot_size_y
        station_marker.scale.z = 0.01
        station_marker.color.a = 1.0
        station_marker.color.r = 0.0
        station_marker.color.g = 1.0
        station_marker.color.b = 0.0
        marker_array.markers.append(station_marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)
        self.get_logger().info(f"Published visualization markers.")

def main(args=None):
    rclpy.init(args=args)
    node = MapGridVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
