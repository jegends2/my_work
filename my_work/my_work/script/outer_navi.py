import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import asyncio

class AutomatedWaypointGenerator(Node):
    def __init__(self):
        super().__init__('automated_waypoint_generator')

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=10)
        )

        self.waypoint_publisher = self.create_publisher(PoseStamped, '/goal_pose', QoSProfile(depth=10))
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.map_data = None
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

        self.current_waypoint_index = -1
        self.waypoints = []

        self.get_logger().info('Automated Waypoint Generator initialized')

    def map_callback(self, msg):
        self.map_data = msg
        self.update_bounds()
        self.generate_waypoints()

    def update_bounds(self):
        if self.map_data:
            info = self.map_data.info
            width = info.width
            height = info.height
            resolution = info.resolution
            origin = info.origin

            self.x_min = origin.position.x
            self.x_max = origin.position.x + width * resolution
            self.y_min = origin.position.y
            self.y_max = origin.position.y + height * resolution

            self.process_map_data()

    def process_map_data(self):
        # 장애물 감지 및 경계선 내의 영역 계산 (옵션)
        pass

    def generate_waypoints(self):
        if not self.map_data:
            return

        self.waypoints = []
        step_size = 1.0  # Waypoint 간격 조정 가능
        iterations = 3  # 사각형 레벨 조정

        for i in range(iterations):
            x_min = self.x_min + i * step_size
            x_max = self.x_max - i * step_size
            y_min = self.y_min + i * step_size
            y_max = self.y_max - i * step_size

            if x_min >= x_max or y_min >= y_max:
                break

            waypoints = self.generate_square_waypoints(x_min, x_max, y_min, y_max)
            self.waypoints.extend(waypoints)

        self.current_waypoint_index = 0
        self.publish_next_waypoint()

    def generate_square_waypoints(self, x_min, x_max, y_min, y_max):
        waypoints = []
        num_points = 4

        for i in range(num_points):
            x = x_min if i % 2 == 0 else x_max
            y = y_min if i < 2 else y_max

            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.orientation.w = 1.0
            waypoints.append(waypoint)

        return waypoints

    async def wait_for_waypoint_reached(self):
        self.get_logger().info('Waiting for waypoint to be reached...')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints[self.current_waypoint_index:]
        future = self.action_client.send_goal_async(goal_msg)

        try:
            result = await future
            if result.status == 2:  # 2 = SUCCEEDED
                self.get_logger().info('Waypoint reached successfully.')
                self.current_waypoint_index += 1
                self.publish_next_waypoint()
            else:
                self.get_logger().error('Failed to reach waypoint.')
        except Exception as e:
            self.get_logger().error(f'Error waiting for waypoint: {e}')

    def publish_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            self.waypoint_publisher.publish(waypoint)
            self.get_logger().info(f'Publishing waypoint: {waypoint.pose.position.x}, {waypoint.pose.position.y}')

            # asyncio 작업을 직접 실행
            loop = asyncio.get_event_loop()
            loop.create_task(self.wait_for_waypoint_reached())
        else:
            self.get_logger().info('All waypoints reached or no more waypoints.')

def main(args=None):
    rclpy.init(args=args)
    waypoint_generator = AutomatedWaypointGenerator()

    try:
        loop = asyncio.get_event_loop()
        while rclpy.ok():
            rclpy.spin_once(waypoint_generator)
            loop.run_until_complete(asyncio.sleep(0.1))  # Prevent CPU overuse and allow asyncio tasks to run
    finally:
        waypoint_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
