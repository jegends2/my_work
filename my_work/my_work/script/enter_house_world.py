import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class EnterHouseNode(Node):

    def __init__(self):
        super().__init__('enter_house_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        self._action_client.wait_for_server()

        # Define the pose to navigate to
        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = 'map'
        self._goal_pose.pose.position.x = 3.0  # Example x position
        self._goal_pose.pose.position.y = 1.0  # Example y position
        self._goal_pose.pose.orientation.w = 0.0  # Example orientation

        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose
        self.get_logger().info('Sending goal pose to navigate to...')
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = EnterHouseNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
