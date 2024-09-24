import cv2
import numpy as np

def draw_grid_on_white(image, grid_size, white_mask):
    """
    흰색 부분에 대해서만 그리드화를 적용하고, 각 사각형에 숫자 라벨을 붙입니다.
    """
    height, width = image.shape[:2]
    color = (204, 153, 255)  # 연 보라색 경계선
    label_color = (0, 255, 0)  # 초록색 라벨
    font_scale = 0.5
    font_thickness = 1
    font = cv2.FONT_HERSHEY_SIMPLEX

    label_id = 1

    # 그리드 그리기 및 라벨 붙이기
    for y in range(0, height, grid_size):
        for x in range(0, width, grid_size):
            if np.any(white_mask[y:y+grid_size, x:x+grid_size] == 255):
                cv2.rectangle(image, (x, y), (x + grid_size, y + grid_size), color, 2)
                # 라벨 위치 계산 (사각형의 중앙)
                label_position = (x + grid_size // 2, y + grid_size // 2)
                cv2.putText(image, str(label_id), label_position, font, font_scale, label_color, font_thickness, cv2.LINE_AA)
                label_id += 1

    return image

def process_image(image_path, grid_size):
    # 이미지 읽기 (그레이스케일)
    gray_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if gray_image is None:
        print(f"이미지를 로드할 수 없습니다: {image_path}")
        return

    # 흰색 픽셀의 임계값 설정
    white_threshold = 200
    # 흰색 부분만 검출
    white_mask = (gray_image >= white_threshold).astype(np.uint8) * 255

    # 검은색으로 나머지 부분 변경
    processed_image = np.zeros((gray_image.shape[0], gray_image.shape[1], 3), dtype=np.uint8)  # 컬러 이미지 생성
    processed_image[white_mask == 255] = (255, 255, 255)  # 흰색 부분만 흰색으로 설정

    # 정사각형 그리드 추가
    grid_size_pixels = int(grid_size / 0.05)  # 맵의 해상도에 따라 픽셀 단위로 변환
    grid_image = draw_grid_on_white(processed_image.copy(), grid_size_pixels, white_mask)

    # 결과 이미지 시각화
    cv2.imshow('Grid Image with Labels', grid_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    # 이미지 경로 설정
    image_path = '/home/adsol/map.pgm'
    grid_size = 0.6  # 정사각형의 사이즈 (미터 단위)

    process_image(image_path, grid_size)

if __name__ == "__main__":
    main()
