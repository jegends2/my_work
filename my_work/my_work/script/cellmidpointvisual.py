import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json

class Rviz2Visualizer(Node):
    def __init__(self):
        super().__init__('rviz2_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.load_data()

    def load_data(self):
        # JSON 파일에서 midpoints 데이터 로드
        with open('cell_midpoint.json', 'r') as f:
            data = json.load(f)
        self.midpoints = data['midpoints']

    def publish_markers(self):
        marker_array = MarkerArray()

        # 중점을 위한 마커 생성
        for i, midpoint in enumerate(self.midpoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "midpoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(midpoint['x'])
            marker.pose.position.y = float(midpoint['y'])
            marker.pose.position.z = 0.0  # z는 0으로 설정
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2  # 구의 크기
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)

        # 마커 배열을 퍼블리시
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Rviz2Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
