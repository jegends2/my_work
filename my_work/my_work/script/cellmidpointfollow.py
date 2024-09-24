import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import json
import math
from action_msgs.msg import GoalStatus

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(1.0, self.check_and_send_goal)
        self.load_data()
        self.current_index = -1
        self.current_position = None
        self.current_goal_handle = None
        self.goal_status = None  # Store goal status

    def load_data(self):
        with open('cell_midpoint_hallway.json', 'r') as f:
            data = json.load(f)
        self.midpoints = data['midpoints']
        self.available_midpoints = self.midpoints.copy()

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        self.get_logger().info(f'Current position: ({self.current_position.position.x}, {self.current_position.position.y})')

    def check_and_send_goal(self):
        if not self.current_position:
            return

        if self.current_goal_handle is None:
            if not self.available_midpoints:
                self.get_logger().info('No more midpoints available.')
                self.timer.cancel()
                return

            closest_index = self.find_closest_midpoint()
            if closest_index is not None:
                self.current_index = closest_index
                self.send_goal(self.available_midpoints[self.current_index])
                self.available_midpoints.pop(self.current_index)
        else:
            if self.goal_status is not None and self.goal_status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal reached.')
                self.current_goal_handle = None
                self.goal_status = None

    def find_closest_midpoint(self):
        if not self.current_position or not self.available_midpoints:
            return None

        min_distance = float('inf')
        closest_index = None
        
        current_x = self.current_position.position.x
        current_y = self.current_position.position.y
        
        for i, midpoint in enumerate(self.available_midpoints):
            distance = math.sqrt((midpoint['x'] - current_x) ** 2 + (midpoint['y'] - current_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index

    def send_goal(self, midpoint):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = midpoint['x']
        pose.pose.position.y = midpoint['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose
        self.get_logger().info(f'Sending goal to position ({pose.pose.position.x}, {pose.pose.position.y})')

        self._action_client.wait_for_server()
        goal_future = self._action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result:
            self.current_goal_handle = result
            self.goal_status = GoalStatus.STATUS_UNKNOWN  # Initialize status
            self.get_logger().info('Goal accepted.')
            self.check_goal_status()
        else:
            self.get_logger().error('Failed to send goal.')

    def check_goal_status(self):
        if self.current_goal_handle is not None:
            goal_result_future = self.current_goal_handle.get_result_async()
            goal_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()
            if result:
                self.goal_status = result.status
            else:
                self.get_logger().error('Failed to get goal result.')
        except Exception as e:
            self.get_logger().error(f'Exception while getting goal result: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
