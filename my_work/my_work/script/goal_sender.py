import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)  # Change topic name to 'goal_pose'
        self.get_logger().info("Ready to send goals.")
        self.get_goal_from_input()

    def get_goal_from_input(self):
        while True:
            command = input("Enter 'x y' to set goal, or 'quit' to exit: ")
            if command.lower() == 'quit':
                self.get_logger().info("Exiting.")
                break
            else:
                try:
                    x, y = map(float, command.split())
                    self.send_goal(x, y)
                except ValueError:
                    self.get_logger().error("Invalid input. Please enter 'x y' or 'quit'.")

    def send_goal(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0  # Fixed z-coordinate
        goal_msg.pose.orientation.w = 1.0  # Default orientation

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Publishing goal: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
