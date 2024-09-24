import rclpy
from rclpy.node import Node
import subprocess

class MainCJG(Node):

    def __init__(self):
        super().__init__('main_cjg_node')

    def run(self):
        while rclpy.ok():
            task = input("Enter task (enter/return/wall): ").strip()
            if task == 'enter':
                self.get_logger().info("Starting enter house task...")
                self.execute_enter_task()
            elif task == 'return':
                self.get_logger().info("Starting return task...")
                self.execute_return_task()
            elif task == 'wall':
                self.get_logger().info("Starting wall navigation task...")
                self.execute_wall_task()
            else:
                self.get_logger().error("Invalid task. Please enter 'enter', 'return', or 'wall'.")

    def execute_enter_task(self):
        # Launch the enter_house_world.py node
        self.get_logger().info("Launching enter_house_world.py node...")
        subprocess.Popen(['ros2', 'run', 'my_work', 'enter_house_world'])

    def execute_return_task(self):
        # Launch the return_station_world.py node
        self.get_logger().info("Launching return_station_world.py node...")
        subprocess.Popen(['ros2', 'run', 'my_work', 'return_station_world'])

    def execute_wall_task(self):
        # Launch the wall_navi_world.py node with a start point
        self.get_logger().info("Launching wall_navi_world.py node...")
        subprocess.Popen(['ros2', 'run', 'my_work', 'wall_navi_world', 'station_x', 'station_y'])

def main(args=None):
    rclpy.init(args=args)
    node = MainCJG()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
