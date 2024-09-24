import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
from rclpy.executors import MultiThreadedExecutor

class HouseCleaningNavigator(Node):
    def __init__(self):
        super().__init__('house_cleaning_navigator')

        # Action client for NavigateToPose
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define the station position (starting point)
        self.station_position = (-4.5, -3.0)

        # Define the waypoints (corners of the shrunken map)
        self.waypoints = self.generate_shrunken_map_waypoints()

        # Sort waypoints based on distance from the station
        self.sort_waypoints_by_distance()

        # Index of the current waypoint
        self.current_waypoint_index = 0

        # Start navigation
        self.send_next_goal()

    def generate_shrunken_map_waypoints(self):
        # Define original corners within the wall boundaries
        original_corners = [
            (-4.5, -3), (-4.5, 5), (9, 5), (9, -4)
        ]

        # Shrink the map by 0.5m within the wall boundaries
        shrink_distance = 0.5
        shrunken_corners = [
            (x + shrink_distance if x < 0 else x - shrink_distance,
             y + shrink_distance if y < 0 else y - shrink_distance)
            for x, y in original_corners
        ]

        return shrunken_corners

    def sort_waypoints_by_distance(self):
        def distance_from_station(waypoint):
            return math.sqrt((waypoint[0] - self.station_position[0]) ** 2 + (waypoint[1] - self.station_position[1]) ** 2)

        self.waypoints.sort(key=distance_from_station)

    def send_next_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints visited. Returning to station.')
            self.send_goal(self.station_position)
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending goal to waypoint: {waypoint}')

        self.send_goal(waypoint)

    def send_goal(self, position):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = position[0]
        goal_pose.pose.position.y = position[1]
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected.')
            return

        self.get_logger().info('Goal sent successfully.')
        self.get_logger().info('Waiting for result...')
        result_future = self.navigate_to_pose_client._get_result_async(goal_handle)
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 3:  # Status 3 indicates the goal was successfully reached.
            self.get_logger().info('Reached the waypoint successfully.')
            # Move to the next waypoint
            self.current_waypoint_index += 1
            self.send_next_goal()
        else:
            self.get_logger().error('Failed to reach the waypoint.')

def main(args=None):
    rclpy.init(args=args)
    node = HouseCleaningNavigator()

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
