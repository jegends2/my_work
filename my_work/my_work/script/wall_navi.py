import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus  # 수정된 부분
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped

class WallNavigator(Node):

    def __init__(self, layer_number):
        super().__init__('wall_navigator')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize current pose
        self.current_pose = None

        # Action Client
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define waypoints based on the layer
        self.waypoints = self.define_waypoints(layer_number)

        # Start navigation process
        self.current_waypoint_index = 0
        self.navigate_to_next_waypoint()

    def define_waypoints(self, layer_number):
        if layer_number == 1:
            return [
                (-1.5, 2.0), (2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 1.5)
            ]
        elif layer_number == 2:
            return [
                (-1.5, 1.5), (1.5, 1.5), (1.5, -1.5), (-1.5, -1.5)
            ]
        elif layer_number == 3:
            return [
                (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)
            ]
        elif layer_number == 4:
            return [
                (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)
            ]
        else:
            self.get_logger().error("Invalid layer number")
            return []

    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached")
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: ({goal_x}, {goal_y})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(goal_x)
        goal_msg.pose.pose.position.y = float(goal_y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: {goal_msg.pose.pose}")

        self.navigate_to_pose_client.wait_for_server()
        future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            status = result_future.result().status
            self.get_logger().info(f"Result status: {status}")

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}")
                self.current_waypoint_index += 1
                self.navigate_to_next_waypoint()
            else:
                self.get_logger().error(f"Failed to reach waypoint {self.current_waypoint_index + 1}. Status: {status}")
                self.get_logger().error("Check if the robot is within reach, ensure there are no obstacles, and verify the goal is achievable.")
        else:
            self.get_logger().error("Failed to send navigation goal")

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.current_pose = transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF Exception: {e}")

def main(args=None):
    rclpy.init(args=args)

    layer_number = int(input("Enter layer number (1-4): "))
    wall_navigator_node = WallNavigator(layer_number)

    rclpy.spin(wall_navigator_node)
    wall_navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
