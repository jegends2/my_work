import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np

def read_costmap_from_xml(filename='/home/adsol/costmap.xml'):
    tree = ET.parse(filename)
    root = tree.getroot()

    # 해상도 및 원점 읽기
    resolution = float(root.find('resolution').text)
    origin = list(map(float, root.find('origin').text.split()))

    # 데이터 읽기
    data = []
    for row_elem in root.find('data').findall('row'):
        row = list(map(int, row_elem.text.split()))
        data.append(row)
    
    data = np.array(data, dtype=np.uint8)
    return data, resolution, origin

def plot_costmap(data, resolution, origin):
    plt.imshow(data, cmap='gray', origin='lower')
    plt.title(f'Costmap (Resolution: {resolution}, Origin: {origin})')
    plt.colorbar(label='Cost Value')
    plt.show()

# 예시 사용법
data, resolution, origin = read_costmap_from_xml('costmap.xml')
plot_costmap(data, resolution, origin)
