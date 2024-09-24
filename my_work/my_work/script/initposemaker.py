import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        # QoS 설정
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABLE
        )
        
        # 퍼블리셔 설정
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_profile)
        
        # 서브스크라이버 설정
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile
        )

        self.scan_data = None
        self.map_data = None

        # tf2 브로드캐스터 및 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg):
        self.scan_data = msg
        self.set_initial_pose()

    def map_callback(self, msg):
        self.map_data = msg

    def set_initial_pose(self):
        if self.scan_data is None or self.map_data is None:
            return

        # 초기 위치를 계산하는 부분
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # 여기에 스캔과 맵 데이터를 기반으로 초기 위치를 설정하는 로직을 추가합니다.
        # 현재는 예시로 (0, 0, 0) 위치와 (0, 0, 0, 1) 쿼터니언을 사용합니다.
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        
        # tf2를 사용하여 쿼터니언 생성
        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info('Initial pose set to (0, 0, 0) with orientation (0, 0, 0, 1)')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
