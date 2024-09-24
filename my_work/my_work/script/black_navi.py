import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile

class BlackNavigator(Node):
    def __init__(self):
        super().__init__('black_navigator')

        # 웨이포인트 파일 로드
        self.waypoints = self.load_waypoints("waypoints.txt")
        self.current_index = 0

        # 웨이포인트 퍼블리셔 설정
        self.path_pub = self.create_publisher(Path, 'path', QoSProfile(depth=10))

        # 타이머 설정: 1초마다 웨이포인트 전송
        self.timer = self.create_timer(1.0, self.publish_waypoint)

    def load_waypoints(self, file_path):
        waypoints = []
        try:
            with open(file_path, "r") as file:
                for line in file:
                    x, y = map(float, line.strip().split(','))
                    waypoints.append((x, y))
        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} not found")
        return waypoints

    def publish_waypoint(self):
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            path_msg = Path()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.w = 1.0  # 기본 방향
            path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published waypoint: {waypoint}")

            self.current_index += 1
        else:
            self.timer.cancel()  # 모든 웨이포인트를 완료하면 타이머를 취소
            self.get_logger().info("All waypoints published")

def main(args=None):
    rclpy.init(args=args)
    node = BlackNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
