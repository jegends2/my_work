import cv2
import numpy as np
import argparse

def generate_waypoints(input_path, output_path, grid_size=20):
    # PGM 파일을 열기
    image_data = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

    if image_data is None:
        raise ValueError(f"Image {input_path} not found or is not valid.")
    
    # 흰색 부분 찾기
    white_pixels = (image_data == 255)
    coords = np.argwhere(white_pixels)
    
    # 웨이포인트를 저장할 리스트
    waypoints = []

    # 그리드 사이즈 설정
    grid_size_px = int(grid_size)

    for y in range(0, image_data.shape[0], grid_size_px):
        for x in range(0, image_data.shape[1], grid_size_px):
            # 그리드 영역의 중앙 좌표 계산
            center_x = x + grid_size_px // 2
            center_y = y + grid_size_px // 2

            # 중앙 좌표가 이미지 범위 내에 있는지 확인
            if (center_x < image_data.shape[1]) and (center_y < image_data.shape[0]):
                waypoints.append((center_x, center_y))
                cv2.circle(image_data, (center_x, center_y), 2, (0, 255, 0), -1)

    # 결과를 저장
    cv2.imwrite(output_path, image_data)
    
    # 웨이포인트 파일 저장
    with open("waypoints.txt", "w") as f:
        for (x, y) in waypoints:
            f.write(f"{x},{y}\n")

    print(f"Waypoints saved to waypoints.txt")

def main():
    parser = argparse.ArgumentParser(description='Generate grid waypoints from a PGM image.')
    parser.add_argument('input', type=str, help='Input PGM file path')
    parser.add_argument('output', type=str, help='Output PGM file path')
    parser.add_argument('--grid_size', type=float, default=20.0, help='Grid size in pixels')
    args = parser.parse_args()

    generate_waypoints(args.input, args.output, args.grid_size)

if __name__ == "__main__":
    main()
