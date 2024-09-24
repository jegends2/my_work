import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import json
import time

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')
        
        # Action Client를 초기화
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 경로 로딩
        self._load_path('path_try.json')
        
        # 목표 도달 여부 확인
        self._goal_reached = False
        
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
        self._wait_for_goal()

    def _wait_for_goal(self):
        while not self._goal_reached and rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.5)

        if self._goal_reached:
            self.get_logger().info('Goal reached. Sending next goal.')
            self._current_goal_index += 1
            if self._current_goal_index < len(self._path['points']):
                self._send_goal(self._path['points'][self._current_goal_index])
            else:
                self.get_logger().info('All goals completed.')
    
    def _feedback_callback(self, feedback_msg):
        # 피드백 메시지에서 현재 위치 정보를 확인하고 출력합니다.
        current_pose = feedback_msg.feedback.current_pose.pose
        self.get_logger().info(f'Feedback: {current_pose}')
        
        goal_pose = self._path['points'][self._current_goal_index]
        goal_tolerance = 0.35  #0.5  # Goal tolerance 값을 설정합니다.
        
        if self._is_goal_reached(current_pose, goal_pose, goal_tolerance):
            self.get_logger().info('Goal reached.')
            self._goal_reached = True
        else:
            self.get_logger().info('Goal is active and still being processed.')

    def _is_goal_reached(self, current_pose, goal_pose, tolerance):
        distance = ((current_pose.position.x - goal_pose['x']) ** 2 +
                    (current_pose.position.y - goal_pose['y']) ** 2) ** 0.5
        return distance < tolerance

def main(args=None):
    rclpy.init(args=args)
    node = PathNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
