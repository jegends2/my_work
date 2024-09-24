import rclpy
from rclpy.node import Node
from fields2cover_ros.msg import PathCoverageResult
from fields2cover_ros.srv import PlanCoverage

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.cli = self.create_client(PlanCoverage, 'plan_coverage')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = PlanCoverage.Request()
        # Configure the request parameters (e.g., polygon points)
        self.req.polygon_points = [(x, y) for x, y in your_polygon_points]
        self.future = self.cli.call_async(self.req)

    def plan_coverage_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Coverage Path: %s' % response.path)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    coverage_planner = CoveragePlanner()
    rclpy.spin(coverage_planner)
    coverage_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
