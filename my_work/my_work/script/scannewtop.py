#import os
import cv2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
#import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
import math

class LaserScanToImage(Node):
    def __init__(self):
        super().__init__('laser_scan_to_image')

       # os.system("ros2 run tf2_ros static_transform_publisher 0 0 0 0 0.785 0 laser_frame base_link")


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(Image, '/scan_image', qos_profile)
        self.bridge = CvBridge()

        self.rotation_angle = 47.0  # Rotate by 45 degrees


    def listener_callback(self, msg):
        # Convert LaserScan to Image
        scan_ranges = np.array(msg.ranges)
        scan_ranges[scan_ranges == 0.0] = np.nan  # Set invalid ranges to NaN

        # Create an image where the scan ranges will be visualized
        image = np.zeros((480, 640, 3), dtype=np.uint8
)
        angle_rad = math.radians(self.rotation_angle)
        rotation_matrix = np.array([
            [math.cos(angle_rad), -math.sin(angle_rad)],
            [math.sin(angle_rad), math.cos(angle_rad)] 
        ])

        for i in range(len(scan_ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if not np.isnan(scan_ranges[i]):
                x = int(320 + scan_ranges[i] * 100 * np.cos(angle))
                y = int(240 + scan_ranges[i] * 100 * np.sin(angle))
 
                rotated_coords = np.dot(rotation_matrix, np.array([x - 320, y - 240]))
                x_rotated = int(rotated_coords[0] + 320)
                y_rotated = int(rotated_coords[1] + 240)

                if 0 <= x_rotated < 640 and 0 <= y_rotated < 480:
                    image[y_rotated, x_rotated] = [255, 255, 255]


#               if 0 <= x < 640 and 0 <= y < 480:
 #                   image[y, x] = [255, 255, 255]


        cv2.line(image, (320, 0), (320, 480), (0, 255, 0), 1)  # Y axis in green
        cv2.line(image, (0, 240), (640, 240), (0, 255, 0), 1)  # X axis in green
        cv2.circle(image, (320, 240), 5, (0, 0, 255), -1)       # Origin in red

        # Convert image to ROS message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

