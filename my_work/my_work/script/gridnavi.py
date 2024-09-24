import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = []  # List of waypoints to visit

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal...')
        self._action_client.send_goal_async(goal_msg)

    def plan_and_execute(self):
        # Example waypoints; replace with actual calculated waypoints
        self.waypoints = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]  # Example waypoints

        for (x, y) in self.waypoints:
            self.send_goal(x, y)
            rclpy.spin_until_future_complete(self, self._action_client.get_result_async())

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()

    # Define the start point (station) and send it as the first goal
    start_point = (0.0, 0.0)  # Station coordinates
    waypoint_navigator.send_goal(*start_point)
    rclpy.spin_until_future_complete(waypoint_navigator, waypoint_navigator._action_client.get_result_async())

    # Plan and execute the route to other waypoints
    waypoint_navigator.plan_and_execute()

    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
