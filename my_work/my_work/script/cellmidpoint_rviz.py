import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import json

class Rviz2Visualizer(Node):
    def __init__(self):
        super().__init__('rviz2_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.state_subscriber = self.create_subscription(Int32, 'cell_state_change', self.state_callback, 10)
        self.midpoints = self.load_midpoints()
        self.cell_states = {i: True for i in range(len(self.midpoints))}
        self.create_timer(1.0, self.publish_markers)

    def load_midpoints(self):
        with open('cell_midpoint.json', 'r') as f:
            data = json.load(f)
        return data['midpoints']

    def state_callback(self, msg):
        cell_id = msg.data
        if cell_id in self.cell_states:
            self.cell_states[cell_id] = not self.cell_states[cell_id]
            self.publish_markers()  # Update markers when state changes

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, midpoint in enumerate(self.midpoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "midpoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(midpoint['x']), y=float(midpoint['y']), z=0.0)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0 if not self.cell_states.get(i, True) else 0.0
            marker.color.g = 0.0 if not self.cell_states.get(i, True) else 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Rviz2Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
