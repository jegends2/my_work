import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Waiting for action server...')
        
        # Load path points from JSON
        self.path_points = self.load_path_from_json()
        self.current_point_index = 0

    def load_path_from_json(self):
        with open('headland_polygon.json', 'r') as f:
            data = json.load(f)
        polygons = data['polygons']
        if polygons:
            coords = polygons[0]
            # Ensure path points are valid
            return [(float(coord[0]), float(coord[1])) for coord in coords]
        return []

    def timer_callback(self):
        if not self.path_points or self.current_point_index >= len(self.path_points):
            self.get_logger().info('Path completed or no path points available.')
            return

        # Get the current point
        point = self.path_points[self.current_point_index]
        self.current_point_index += 1
        
        # Send NavigateToPose action
        self.send_goal(point)

    def send_goal(self, point):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0  # Default orientation
        
        goal_msg.pose = pose
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info('Goal accepted!')
        else:
            self.get_logger().info('Goal rejected.')

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
