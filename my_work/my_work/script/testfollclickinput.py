import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
import json
import time

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')

        # Action Client를 초기화
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 사용자 입력을 받아 반복 횟수 설정
        self._loop_count = self._get_loop_count()
        self._current_loop = 0

        # 경로 로딩
        self._load_path('path_try.json')

        # 목표 도달 여부 확인
        self._goal_reached = False
        self._goal_failed = False  # 목표 실패 여부 추가
        self._retry_count = 0  # 실패 시 재시도 횟수

    def _get_loop_count(self):
        # 사용자로부터 반복 횟수 입력받기
        while True:
            try:
                loop_count = int(input("Enter the number of loops: "))
                if loop_count > 0:
                    return loop_count
                else:
                    print("Please enter a positive integer.")
            except ValueError:
                print("Invalid input. Please enter a valid integer.")

    def _load_path(self, file_name):
        with open(file_name, 'r') as f:
            self._path = json.load(f)

        # 첫 번째 포인트를 목표로 설정
        if len(self._path['points']) > 0:
            self._current_goal_index = 0
            self._send_goal(self._path['points'][self._current_goal_index])
        else:
            self.get_logger().error('No points found in the path file.')

    def _send_goal(self, goal_point):
        self.get_logger().info(f'Sending goal: {goal_point}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = Header(frame_id='map')
        goal_msg.pose.pose.position = Point(x=goal_point['x'], y=goal_point['y'], z=0.0)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        
        self.get_logger().info('Goal accepted.')
        self._goal_reached = False
        self._goal_failed = False
        self._wait_for_goal()

    def _wait_for_goal(self):
        while not (self._goal_reached or self._goal_failed) and rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.5)

        if self._goal_reached:
            self._retry_count = 0  # 목표에 도달했으면 재시도 횟수 초기화
            self._handle_goal_success()
        elif self._goal_failed:
            self._retry_count += 1  # 실패 시 재시도 카운트 증가
            if self._retry_count < 3:  # 3번까지 재시도 허용
                self.get_logger().info(f'Retrying goal, attempt {self._retry_count}')
                self._send_goal(self._path['points'][self._current_goal_index])
            else:
                self.get_logger().info(f'Skipping goal after {self._retry_count} attempts.')
                self._retry_count = 0  # 재시도 카운트 초기화
                self._handle_goal_failure()

    def _handle_goal_success(self):
        if self._current_goal_index + 1 < len(self._path['points']):
            self._current_goal_index += 1
            self._send_goal(self._path['points'][self._current_goal_index])
        else:
            self._handle_loop_completion()

    def _handle_goal_failure(self):
        if self._current_goal_index + 1 < len(self._path['points']):
            self._current_goal_index += 1
            self._send_goal(self._path['points'][self._current_goal_index])
        else:
            self._handle_loop_completion()

    def _handle_loop_completion(self):
        self._current_loop += 1
        if self._current_loop < self._loop_count:
            self.get_logger().info('Loop completed. Restarting...')
            self._current_goal_index = 0
            self._send_goal(self._path['points'][self._current_goal_index])
        else:
            self.get_logger().info('All loops completed. Waiting for input.')
            self._wait_for_input()

    def _feedback_callback(self, feedback_msg):
        current_pose = feedback_msg.feedback.current_pose.pose
        goal_pose = self._path['points'][self._current_goal_index]
        goal_tolerance = 0.35  # Goal tolerance 값을 설정합니다.

        # 피드백 메시지 로그 추가
        self.get_logger().info(f'Current Pose: {current_pose.position.x}, {current_pose.position.y}')
        self.get_logger().info(f'Goal Pose: {goal_pose["x"]}, {goal_pose["y"]}')

        if self._is_goal_reached(current_pose, goal_pose, goal_tolerance):
            self.get_logger().info('Stopped')
            self._goal_reached = True
        else:
            self.get_logger().info('Moving')

    def _is_goal_reached(self, current_pose, goal_pose, tolerance):
        distance = ((current_pose.position.x - goal_pose['x']) ** 2 +
                    (current_pose.position.y - goal_pose['y']) ** 2) ** 0.5
        return distance < tolerance

    def _wait_for_input(self):
        # 사용자로부터 입력을 기다리며 터미널을 대기 상태로 둡니다.
        input("Press Enter to start a new session...")

def main(args=None):
    rclpy.init(args=args)
    node = PathNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
