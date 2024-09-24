import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapProcessor(Node):
    def __init__(self):
        super().__init__('map_processor')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # 퍼블리셔 설정
        self.publisher_filtered = self.create_publisher(OccupancyGrid, '/visualization_map_array', 10)
        self.publisher_grid = self.create_publisher(OccupancyGrid, '/visualization_grid_array', 10)

        self.grid_size = 20  # 그리드 크기 20cm x 20cm

    def map_callback(self, msg):
        map_width = msg.info.width
        map_height = msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape((map_height, map_width))

        # 실시간 필터링: 흰색(255)과 검은색(0) 이외의 색상은 제거
        filtered_map = np.where((map_data > 50) & (map_data < 205), 100, -1)  # 100은 occupied, -1은 unknown으로 설정

        # 필터링된 맵을 퍼블리시
        self.publish_filtered_map(filtered_map, msg.info)

        # 그리드 셀 계산 및 퍼블리시
        grid_centers = self.generate_grids(filtered_map, msg.info)
        self.publish_grid_map(grid_centers, msg.info)

    def publish_filtered_map(self, filtered_map, map_info):
        filtered_msg = OccupancyGrid()
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.header.frame_id = "map"
        filtered_msg.info = map_info
        # 값의 범위를 -128에서 127로 제한
        filtered_msg.data = filtered_map.astype(np.int8).flatten().tolist()

        self.publisher_filtered.publish(filtered_msg)
        self.get_logger().info('Filtered map published.')

    def generate_grids(self, filtered_map, map_info):
        height, width = filtered_map.shape
        grid_centers = []

        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        resolution = map_info.resolution

        for y in range(0, height, self.grid_size):
            for x in range(0, width, self.grid_size):
                grid = filtered_map[y:y+self.grid_size, x:x+self.grid_size]
                if np.all(grid == 100):  # 흰색 영역만 선택
                    center_x = origin_x + (x + self.grid_size / 2) * resolution
                    center_y = origin_y + (y + self.grid_size / 2) * resolution
                    grid_centers.append((center_x, center_y))
        
        return grid_centers

    def publish_grid_map(self, grid_centers, map_info):
        grid_map = np.full((map_info.height, map_info.width), -1, dtype=np.int8)  # 기본값을 -1로 설정

        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        resolution = map_info.resolution

        for center in grid_centers:
            map_x = int((center[0] - origin_x) / resolution)
            map_y = int((center[1] - origin_y) / resolution)
            if 0 <= map_x < map_info.width and 0 <= map_y < map_info.height:
                grid_map[map_y, map_x] = 100  # 그리드 중심을 흰색으로 설정
        
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info = map_info
        grid_msg.data = grid_map.flatten().tolist()

        self.publisher_grid.publish(grid_msg)
        self.get_logger().info('Grid map published.')

def main(args=None):
    rclpy.init(args=args)
    map_processor = MapProcessor()
    rclpy.spin(map_processor)
    map_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
