import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

class SequentialNavigator(Node):
    def __init__(self):
        super().__init__('sequential_navigator')

        # Initialize Action Client for NavigateToPose
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define the waypoints
        self.waypoints = [
            (-4.5, -3), (-4.5, 5), (1, 5), (1, 1), (9, 1),
            (9, 5), (3, 5), (3, 2), (3, 5), (9, 5), (9, -0.25),
            (9, -4), (7.5, -4), (7.5, -0.25), (7, 1), (-2.5, 1),
            (-3, 3), (-4, 3), (-4.5, 3)
        ]

        # Index of the current waypoint
        self.current_waypoint_index = 0

        # Flag to check if the action has been completed
        self.goal_completed = False

        # Start navigation
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints have been visited.')
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending goal to waypoint: {waypoint}')

        # Convert waypoint coordinates to float
        x, y = map(float, waypoint)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # Assume facing forward

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send the goal to the action server
        self.navigate_to_pose_client.wait_for_server()
        self.get_logger().info('Action server is up and running.')
        future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info('Goal sent successfully.')
            self.goal_completed = False
            self.wait_for_result(result)  # Wait for the result
        else:
            self.get_logger().error('Failed to send goal.')

    def wait_for_result(self, goal_handle):
        # Create a future to wait for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result()
            if result:
                self.get_logger().info('Reached the waypoint successfully.')
                self.goal_completed = True
                # Move to the next waypoint
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.send_next_goal()
                else:
                    self.get_logger().info('All waypoints have been visited.')
            else:
                self.get_logger().error('Failed to reach the waypoint.')
        except Exception as e:
            self.get_logger().error(f'Exception while getting result: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SequentialNavigator()

    # Use MultiThreadedExecutor to handle multiple callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
