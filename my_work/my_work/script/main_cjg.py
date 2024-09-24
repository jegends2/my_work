import rclpy
from rclpy.node import Node
import subprocess
import time

class MainCJG(Node):

    def __init__(self):
        super().__init__('main_cjg')
        self.get_logger().info('MainCJG Node Initialized')

    def run(self):
        while rclpy.ok():
            task = input("Enter task (return/wall): ").strip().lower()

            if task == 'return':
                self.get_logger().info('Starting return task...')
                self.run_return_task()
            elif task == 'wall':
                self.get_logger().info('Starting wall navigation task...')
                self.run_wall_task()
            else:
                self.get_logger().info('Invalid task. Please enter "return" or "wall".')

    def run_return_task(self):
        # Start the return_station node
        subprocess.Popen(["ros2", "run", "my_work", "return_station"])
        self.wait_for_completion()

    def run_wall_task(self):
        layer = input("Enter layer number (1-4): ").strip()
        if layer in ['1', '2', '3', '4']:
            # Start the wall_navi node with the specified layer
            subprocess.Popen(["ros2", "run", "my_work", "wall_navi", layer])
            self.wait_for_completion()
        else:
            self.get_logger().info('Invalid layer number. Please enter a number between 1 and 4.')

    def wait_for_completion(self):
        # Wait for a short period to allow the subprocess to start and complete its execution
        self.get_logger().info('Waiting for task to complete...')
        time.sleep(5)  # Adjust sleep time as needed based on task duration

def main(args=None):
    rclpy.init(args=args)
    main_cjg_node = MainCJG()
    main_cjg_node.run()
    main_cjg_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
