import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from map_filtering import MapProcessor
from nav2_simple_commander.robot_navigator import BasicNavigator

class TurtleBotNavigation(Node):
    def __init__(self, grid_centers):
        super().__init__('turtlebot_navigation')
        self.navigator = BasicNavigator()
        self.grid_centers = grid_centers

    def go_to_goal(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)

        # 결과 대기 및 처리
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info('Current task feedback: {0}'.format(feedback))
        
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == BasicNavigator.TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an unknown result!')

    def navigate_to_station(self, station_x, station_y):
        self.go_to_goal(station_x, station_y)

    def explore_grids(self):
        for grid_pos in self.grid_centers:
            self.go_to_goal(grid_pos[0], grid_pos[1])

def main(args=None):
    rclpy.init(args=args)
    
    # Station 좌표 (예시)
    station_x = 0.1
    station_y = 0.1
    
    processor = MapProcessor()
    rclpy.spin_once(processor)  # 맵을 한 번 수신

    navigator = TurtleBotNavigation(processor.grid_centers)
    navigator.navigate_to_station(station_x, station_y)  # Station 구역으로 복귀
    navigator.explore_grids()  # 그리드 탐색
    navigator.navigate_to_station(station_x, station_y)  # 모든 탐색 후 Station 구역으로 복귀

    rclpy.shutdown()

if __name__ == '__main__':
    main()
