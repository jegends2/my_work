import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class ReturnStation(Node):

    def __init__(self):
        super().__init__('return_station')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters for station
        self.station_center_x = -4.5  # Define the station center x
        self.station_center_y = -3.0  # Define the station center y
        self.station_radius = 0.1 # station's radius

        self.current_pose = None

        # Action Client
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback triggered')
        self.get_current_pose()

        if self.current_pose:
            self.get_logger().info(f'Current pose: ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})')
            if not self.is_within_station_area(self.current_pose):
                self.get_logger().info('Navigating to station')
                self.navigate_to_station()
            else:
                self.get_logger().info('Already at the station')
        else:
            self.get_logger().info('Current pose not available')

    def is_within_station_area(self, pose):
        if self.station_center_x is None or self.station_center_y is None:
            return False

        x = pose.pose.position.x
        y = pose.pose.position.y
        distance = ((x - self.station_center_x) ** 2 + (y - self.station_center_y) ** 2) ** 0.5

        return distance <= self.station_radius

    def navigate_to_station(self):
        if self.station_center_x and self.station_center_y:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = self.station_center_x
            goal_msg.pose.pose.position.y = self.station_center_y
            goal_msg.pose.pose.orientation.w = 1.0

            # Send goal
            self.navigate_to_pose_client.wait_for_server()
            self.get_logger().info('Action server is up and running')
            future = self.navigate_to_pose_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info("Navigation to station initiated")
            else:
                self.get_logger().error("Failed to send navigation goal")

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.current_pose = PoseStamped()
            self.current_pose.header.frame_id = 'map'
            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
            self.current_pose.pose.orientation = transform.transform.rotation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    return_station_node = ReturnStation()
    rclpy.spin(return_station_node)
    return_station_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
