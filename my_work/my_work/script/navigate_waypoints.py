import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = self.load_waypoints('waypoints.yaml')
        self._current_waypoint = 0
        self.navigate_to_next_waypoint()

    def load_waypoints(self, filename):
        # Load waypoints from a YAML file or define them here
        # For simplicity, hardcoded in this example
        return [
            {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            {'x': 3.0, 'y': 4.0, 'theta': 1.57},
            {'x': 5.0, 'y': 6.0, 'theta': 3.14},
        ]

    def navigate_to_next_waypoint(self):
        if self._current_waypoint < len(self._waypoints):
            waypoint = self._waypoints[self._current_waypoint]
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = waypoint['x']
            goal_msg.pose.position.y = waypoint['y']
            goal_msg.pose.orientation.z = waypoint['theta']
            
            self._action_client.wait_for_server()
            self._action_client.send_goal_async(goal_msg).add_done_callback(self.on_goal_done)
        else:
            self.get_logger().info('All waypoints visited')

    def on_goal_done(self, future):
        result = future.result()
        self.get_logger().info(f'Waypoint {self._current_waypoint} reached')
        self._current_waypoint += 1
        self.navigate_to_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
