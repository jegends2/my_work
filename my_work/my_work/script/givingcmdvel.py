import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move_forward(self, distance=0.5):
        twist = Twist()
        twist.linear.x = 0.2  # 전진 속도
        duration = distance / abs(twist.linear.x)
        self.publisher_.publish(twist)
        self.get_logger().info(f'{distance}m 전진')
        time.sleep(duration)
        self.stop()

    def move_backward(self, distance=0.5):
        twist = Twist()
        twist.linear.x = -0.2  # 후진 속도
        duration = distance / abs(twist.linear.x)
        self.publisher_.publish(twist)
        self.get_logger().info(f'{distance}m 후진')
        time.sleep(duration)
        self.stop()

    def rotate_left(self, angle=90):
        twist = Twist()
        twist.angular.z = 0.5  # 회전 속도
        duration = angle / abs(twist.angular.z)  # 각도에 따른 시간 계산
        self.publisher_.publish(twist)
        self.get_logger().info(f'{angle}도 왼쪽으로 회전')
        time.sleep(duration)
        self.stop()

    def rotate_right(self, angle=90):
        twist = Twist()
        twist.angular.z = -0.5  # 회전 속도
        duration = angle / abs(twist.angular.z)  # 각도에 따른 시간 계산
        self.publisher_.publish(twist)
        self.get_logger().info(f'{angle}도 오른쪽으로 회전')
        time.sleep(duration)
        self.stop()

    def stop(self):
        twist = Twist()  # 정지
        self.publisher_.publish(twist)
        self.get_logger().info('정지')
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()

    try:
        while True:
            # 사용자로부터 입력을 받음
            command = input("명령을 입력하세요 (left, right, up, down, quit): ").strip().lower()

            if command == "left":
                node.rotate_left()
            elif command == "right":
                node.rotate_right()
            elif command == "up":
                node.move_forward()
            elif command == "down":
                node.move_backward()
            elif command == "quit":
                node.get_logger().info('종료합니다.')
                break
            else:
                node.get_logger().info("유효하지 않은 명령입니다. 다시 입력하세요.")
    except KeyboardInterrupt:
        node.get_logger().info('작업 중단됨')

    node.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
