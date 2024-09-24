import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import sqrt
from tf_transformations import quaternion_from_euler
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._path_publisher = self.create_publisher(Path, '/custom_path', 10)
        self._timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Waiting for action server...')

        # Load path points from JSON
        self.path_points = self.load_path_from_json()
        if not self.path_points:
            self.get_logger().error('No valid path points loaded from JSON.')
            return

        self.current_point_index = 0
        self.goal_sent = False

        # Define a tolerance distance for goal acceptance
        self.goal_tolerance = 3  # Meters tolerance (adjust as needed)

        # Initialize robot position
        self.robot_odom_x = 0.0
        self.robot_odom_y = 0.0

        # Publish the path for visualization
        self.publish_path()

    def load_path_from_json(self):
        try:
            with open('headland_path.json', 'r') as f:
                data = json.load(f)
            
            # Extract points from 'polygons'
            if 'polygons' in data and isinstance(data['polygons'], list) and len(data['polygons']) > 0:
                points = data['polygons'][0]  # Use the first polygon
            else:
                self.get_logger().error('JSON file does not contain valid "polygons" key.')
                return []

            # Check if points are valid
            if not all(len(p) == 2 for p in points):
                self.get_logger().error('Invalid points format in JSON file.')
                return []

            return [(float(p[0]), float(p[1])) for p in points]

        except Exception as e:
            self.get_logger().error(f'Failed to load path from JSON: {e}')
            return []

    def publish_path(self):
        """Publish the path for visualization in RViz."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self._path_publisher.publish(path_msg)

    def timer_callback(self):
        if not self.path_points:
            self.get_logger().info('No path points available.')
            return

        if not self.goal_sent:
            if self.current_point_index >= len(self.path_points):
                self.get_logger().info('All goals completed.')
                return

            point = self.path_points[self.current_point_index]
            self.send_goal(point)
            self.goal_sent = True

        if self.is_goal_reached():
            self.get_logger().info(f'Goal reached! Reached point index: {self.current_point_index}')
            self.goal_sent = False
            self.current_point_index += 1

    def send_goal(self, point):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]

        # Convert Euler angles to quaternion (example angles)
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

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

    def is_goal_reached(self):
        """Check if the robot has reached the goal within tolerance."""
        distance = sqrt(
            (self.robot_odom_x - self.path_points[self.current_point_index][0]) ** 2 +
            (self.robot_odom_y - self.path_points[self.current_point_index][1]) ** 2
        )
        return distance <= self.goal_tolerance

    def odom_callback(self, msg):
        self.robot_odom_x = msg.pose.pose.position.x
        self.robot_odom_y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
