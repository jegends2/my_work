import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = 'map'
        self._goal_pose.header.stamp = self.get_clock().now().to_msg()
        self._callback_group = ReentrantCallbackGroup()
        self._feedback_subscription = self.create_subscription(
            NavigateToPose.Feedback,
            'navigate_to_pose/feedback',
            self.feedback_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=self._callback_group
        )

    def set_goal_pose(self, x, y, theta):
        self._goal_pose.pose.position.x = x
        self._goal_pose.pose.position.y = y
        self._goal_pose.pose.orientation.z = theta

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg}')

    async def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose
        self.get_logger().info(f'Sending goal: {goal_msg}')

        future = self._action_client.send_goal_async(goal_msg)
        result = await future
        if result.code == 3:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().error('Goal failed')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    node.set_goal_pose(-5.0, -3.0, 0.0)  # Example goal pose

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
