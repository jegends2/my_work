import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3

class MarkerControlNode(Node):
    def __init__(self):
        super().__init__('marker_control_node')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        for i in range(10):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "example_namespace"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(i), y=float(i), z=float(0.0))
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
