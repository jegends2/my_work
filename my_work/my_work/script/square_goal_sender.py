import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

class SquareGoalSender(Node):
    def __init__(self):
        super().__init__('square_goal_sender')
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Ready to send goals.")
        self.square_coords = self.get_square_coordinates()
        self.get_logger().info(f"Loaded {len(self.square_coords)} square coordinates.")
        self.show_image_with_squares()  # Display image with squares initially
        self.get_goal_from_input()

    def get_square_coordinates(self):
        # Path to your map image and grid size
        image_path = '/home/adsol/map.pgm'
        grid_size = 0.6  # size of the square in meters

        gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if gray_image is None:
            raise ValueError(f"Unable to load image: {image_path}")

        height, width = gray_image.shape
        origin_x = (width * 0.05) / 2
        origin_y = (height * 0.05) / 2
        resolution = 0.5  # resolution from map.yaml file

        white_threshold = 200
        white_mask = (gray_image >= white_threshold).astype(np.uint8) * 255

        grid_size_pixels = int(grid_size / resolution)
        coordinates = {}
        label_id = 1

        for y in range(0, height, grid_size_pixels):
            for x in range(0, width, grid_size_pixels):
                if np.any(white_mask[y:y+grid_size_pixels, x:x+grid_size_pixels] == 255):
                    map_x = (x + grid_size_pixels // 2) * resolution - origin_x
                    map_y = (y + grid_size_pixels // 2) * resolution - origin_y
                    coordinates[label_id] = (map_x, map_y)
                    label_id += 1

        return coordinates

    def get_goal_from_input(self):
        while rclpy.ok():
            command = input("Enter the square number (e.g., 52) or 'quit' to exit: ")
            if command.lower() == 'quit':
                self.get_logger().info("Exiting.")
                break
            else:
                try:
                    goal_id = int(command)
                    self.publish_goal_pose(goal_id)
                    self.show_highlighted_square(goal_id)
                except ValueError:
                    self.get_logger().error("Invalid input. Please enter a valid number or 'quit' to exit.")

    def publish_goal_pose(self, goal_id):
        if goal_id in self.square_coords:
            x, y = self.square_coords[goal_id]
            self.get_logger().info(f'Publishing goal pose: ID {goal_id}, x={x}, y={y}')
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            self.goal_publisher.publish(pose)
            self.get_logger().info(f'Published goal pose: {pose.pose.position.x}, {pose.pose.position.y}')
        else:
            self.get_logger().error(f'Goal ID {goal_id} not found.')

    def show_image_with_squares(self):
        # Path to your map image and grid size
        image_path = '/home/adsol/map.pgm'
        gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if gray_image is None:
            raise ValueError(f"Unable to load image: {image_path}")

        # Display the image with squares highlighted
        highlighted_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        for (x, y) in self.square_coords.values():
            x_pixel = int((x + (gray_image.shape[1] * 0.05) / 2) / 0.05)
            y_pixel = int((y + (gray_image.shape[0] * 0.05) / 2) / 0.05)
            top_left = (x_pixel - 5, y_pixel - 5)
            bottom_right = (x_pixel + 5, y_pixel + 5)
            cv2.rectangle(highlighted_image, top_left, bottom_right, (0, 255, 0), 2)

        cv2.imshow('Map with Squares', highlighted_image)
        cv2.waitKey(0)

    def show_highlighted_square(self, goal_id):
        if goal_id in self.square_coords:
            image_path = '/home/adsol/map.pgm'
            gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            highlighted_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            x, y = self.square_coords[goal_id]
            x_pixel = int((x + (gray_image.shape[1] * 0.05) / 2) / 0.05)
            y_pixel = int((y + (gray_image.shape[0] * 0.05) / 2) / 0.05)
            top_left = (x_pixel - 10, y_pixel - 10)
            bottom_right = (x_pixel + 10, y_pixel + 10)
            cv2.rectangle(highlighted_image, top_left, bottom_right, (0, 0, 255), 2)
            cv2.imshow('Highlighted Square', highlighted_image)
            cv2.waitKey(0)
        else:
            self.get_logger().error(f'Goal ID {goal_id} not found.')

    def destroy_node(self):
        cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SquareGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
