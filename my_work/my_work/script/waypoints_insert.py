import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WaypointsInserter(Node):
    def __init__(self):
        super().__init__('waypoints_inserter')
        self.publisher = self.create_publisher(Path, 'waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        waypoints = [
            {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            {'x': 3.0, 'y': 4.0, 'theta': 1.57},
            {'x': 5.0, 'y': 6.0, 'theta': 3.14},
        ]

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            # You might want to convert theta to quaternion for orientation
            pose.pose.orientation.z = waypoint['theta']  # Not a correct way to set orientation
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsInserter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

