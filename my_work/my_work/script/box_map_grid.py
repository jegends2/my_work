import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class BoxMapGrid(Node):
    def __init__(self):
        super().__init__('box_map_grid')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "station"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = -2.65
        marker.pose.position.y = -2.65
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher_.publish(marker)
        self.get_logger().info('Published marker at position: (%f, %f, %f)' % 
                              (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))

def main(args=None):
    rclpy.init(args=args)
    node = BoxMapGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
