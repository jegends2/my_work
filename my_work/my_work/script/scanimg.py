import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabiltyPolicy, HistoryPolicy

class LaserScanToImage(Node):
    def __init__(self):
        super().__init__('laser_scan_to_image')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABLE,
            history=HistoryPolicy.RMW_QOS_POLICY_KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile)



    def listener_callback(self, msg):
        # 이미지 크기와 중심 설정
        img_width = 640
        img_height = 480
        img = np.zeros((img_height, img_width), dtype=np.uint8)
        cx = img_width // 2
        cy = img_height // 2

        # 각도와 거리 데이터 변환
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            distance = ranges[i]
            if distance > 0:  # 거리 값이 유효한 경우
                x = int(cx + distance * np.cos(angle))
                y = int(cy + distance * np.sin(angle))
                if 0 <= x < img_width and 0 <= y < img_height:
                    img[y, x] = 255

        # 이미지 저장
        cv2.imwrite('/home/adsol/scan_image.png', img)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
