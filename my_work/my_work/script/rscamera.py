import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
from geometry_msgs.msg import TransformStamped

class DepthToLaserScan(Node):
    def __init__(self):
        super().__init__('depth_to_laser_scan')

        # Depth image subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10
        )

        # LaserScan publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # CvBridge for depth image conversion
        self.bridge = CvBridge()

        # TF 브로드캐스터 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def listener_callback(self, msg):
        # 깊이 이미지를 OpenCV 형식으로 변환
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 깊이 이미지를 레이저 스캔으로 변환
        laser_scan = self.depth_to_laser_scan(depth_image)

        # 레이저 스캔 메시지를 퍼블리시
        self.publisher.publish(laser_scan)

        # TF 퍼블리시
        self.publish_tf()

    def depth_to_laser_scan(self, depth_image):
        # 레이저 스캔 메시지 생성
        scan = LaserScan()

        # 필드 설정
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'  # 레이저 스캔 프레임 (적절히 설정)
        scan.angle_min = -np.pi / 2  # 최소 각도
        scan.angle_max = np.pi / 2    # 최대 각도
        scan.angle_increment = np.pi / 180  # 각도 증가
        scan.range_min = 0.1           # 최소 거리
        scan.range_max = 10.0          # 최대 거리

        # 레이저 스캔 범위 수 설정
        num_ranges = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = []

        for i in range(num_ranges):
            angle = scan.angle_min + i * scan.angle_increment
            # 깊이 이미지를 통해 거리 측정
            distance = self.get_distance_from_depth(depth_image, angle)
            scan.ranges.append(distance)

        return scan

    def get_distance_from_depth(self, depth_image, angle):
        # 각도에 따른 픽셀 좌표 계산
        center_x = depth_image.shape[1] // 2
        center_y = depth_image.shape[0] // 2

        # x, y 픽셀 좌표 계산
        x = int(center_x + (np.cos(angle) * center_y))
        y = int(center_y + (np.sin(angle) * center_y))

        # 범위를 확인하고 거리 값 반환
        if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
            distance = depth_image[y, x]
            return distance / 1000.0  # mm를 m로 변환
        else:
            return float('inf')  # 범위를 벗어난 경우 무한대

    def publish_tf(self):
        # TransformStamped 메시지 생성
        t = TransformStamped()

        # 시간 및 프레임 설정
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # 로봇의 기본 프레임
        t.child_frame_id = 'laser_frame'  # 레이저 스캔 프레임

        # 위치 설정 (카메라 위치가 로봇의 기준에서 어디에 있는지)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 회전 설정 (카메라가 45도 위로 틀어져 있는 상황 반영)
        q = self.quaternion_from_euler(0, np.pi / 4, 0)  # 45도 회전 (X축 기준)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # TF 퍼블리시
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        # 오일러 각을 쿼터니언으로 변환
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    depth_to_laser_scan = DepthToLaserScan()
    rclpy.spin(depth_to_laser_scan)
    depth_to_laser_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
