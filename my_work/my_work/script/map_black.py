import numpy as np

def convert_gray_to_black(input_path, output_path):
    # PGM 파일을 열기
    with open(input_path, 'rb') as f:
        # 파일 포맷 헤더 읽기
        header = f.readline().strip()
        if header != b'P5':
            raise ValueError('Not a PGM P5 file')
        
        # 주석과 너비/높이/최대 값 읽기
        while True:
            line = f.readline().strip()
            if line.startswith(b'#'):
                continue
            else:
                break
        width, height = map(int, line.split())
        max_value = int(f.readline().strip())

        # 이미지 데이터 읽기
        image_data = np.fromfile(f, dtype=np.uint8).reshape((height, width))

    # 연한 회색을 검은색으로 변환
    # 회색 범위를 조정 (연한 회색: 180~250으로 설정)
    gray_min = 180
    gray_max = 250
    image_data[(image_data >= gray_min) & (image_data <= gray_max)] = 0

    # 변환된 이미지를 저장
    with open(output_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode())
        f.write(f'{max_value}\n'.encode())
        image_data.tofile(f)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert gray pixels to black in a PGM file.")
    parser.add_argument('input', type=str, help='Input PGM file path')
    parser.add_argument('output', type=str, help='Output PGM file path')
    args = parser.parse_args()

    convert_gray_to_black(args.input, args.output)
