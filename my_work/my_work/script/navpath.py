import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')

        # Declare and get the path_file parameter with a default value
        self.declare_parameter('path_file', '/home/adsol/path.json')
        self.path_file = self.get_parameter('path_file').get_parameter_value().string_value

        self.path_data = []
        self.current_pose = None
        self.current_goal = None
        self.goal_reached = False

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10))
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.load_path()

    def load_path(self):
        """Load the path from the JSON file."""
        try:
            with open(self.path_file, 'r') as f:
                self.path_data = json.load(f)['path']
            self.get_logger().info(f"Loaded path with {len(self.path_data)} points.")
            self.send_next_pose()
        except IOError as e:
            self.get_logger().error(f"IOError while loading file {self.path_file}: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error while loading file {self.path_file}: {e}")

    def odom_callback(self, msg):
        """Callback to update the current robot pose from Odometry."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            base_link_pose = msg.pose.pose
            self.current_pose = self.transform_pose(base_link_pose, transform)
            self.check_goal_status()
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF Error: {e}")

    def transform_pose(self, pose, transform):
        """Transform a pose from base_link to map frame."""
        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = 'map'
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        # Apply transformation here
        transformed_pose.pose.position.x = pose.position.x + transform.transform.translation.x
        transformed_pose.pose.position.y = pose.position.y + transform.transform.translation.y
        transformed_pose.pose.orientation = pose.orientation
        return transformed_pose.pose

    def send_next_pose(self):
        """Send the next pose in the path."""
        if self.path_data:
            next_point = self.path_data.pop(0)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = next_point['x']
            goal_msg.pose.pose.position.y = next_point['y']
            goal_msg.pose.pose.orientation.w = 1.0

            # Update the current goal for distance calculation
            self.current_goal = goal_msg.pose
            self.goal_reached = False  # Reset goal reached status

            # Send goal and wait for result
            self.action_client.wait_for_server()
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from the action server."""
        result = future.result()
        if result.accepted:
            self.get_logger().info('Goal accepted by action server.')
        else:
            self.get_logger().error('Goal rejected by action server.')

    def feedback_callback(self, feedback):
        """Handle feedback from the action server."""
        # Extract the current robot pose from the feedback
        current_pose = feedback.feedback.current_pose.pose

        # Calculate the remaining distance to the goal
        dx = self.current_goal.pose.position.x - current_pose.position.x
        dy = self.current_goal.pose.position.y - current_pose.position.y
        remaining_distance = (dx**2 + dy**2)**0.5

        self.get_logger().info(f"Remaining distance: {remaining_distance:.2f} meters")

        # Set the goal_reached flag if the remaining distance is small enough
        if remaining_distance < 0.5:  # Adjust this threshold as needed
            self.goal_reached = True

    def check_goal_status(self):
        """Check if the current goal has been achieved."""
        if self.goal_reached:
            self.get_logger().info(f"Reached target pose: ({self.current_goal.pose.position.x}, {self.current_goal.pose.position.y})")
            self.send_next_pose()

def main(args=None):
    rclpy.init(args=args)
    node = PathNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
