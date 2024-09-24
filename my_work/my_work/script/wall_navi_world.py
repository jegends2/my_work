import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

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

        # Publisher for waypoints
        self.marker_publisher = self.create_publisher(MarkerArray, 'waypoints_markers', 10)

        # Start navigation
        self.send_next_goal()
        self.publish_waypoints()

    def send_next_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints have been visited.')
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending goal to waypoint: {waypoint}')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(waypoint[0])  # Ensure waypoint is float
        goal_pose.pose.position.y = float(waypoint[1])  # Ensure waypoint is float
        goal_pose.pose.position.z = 0.0  # Ensure z is float
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send the goal to the action server
        self.navigate_to_pose_client.wait_for_server()
        self.get_logger().info('Action server is up and running.')
        future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            self.get_logger().info('Goal sent successfully.')
            self.get_logger().info('Waiting for result...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')

    def result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info('Reached the waypoint successfully.')
            # Move to the next waypoint
            self.current_waypoint_index += 1
            self.send_next_goal()
        except Exception as e:
            self.get_logger().error(f'Failed to reach the waypoint: {e}')

    def publish_waypoints(self):
        marker_array = MarkerArray()

        for idx, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(waypoint[0])  # Ensure waypoint is float
            marker.pose.position.y = float(waypoint[1])  # Ensure waypoint is float
            marker.pose.position.z = 0.0  # Ensure z is float
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    # Use MultiThreadedExecutor to handle multiple callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
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
