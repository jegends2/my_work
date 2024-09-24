import cv2
import numpy as np
import xml.etree.ElementTree as ET
import yaml

def read_pgm_and_yaml(pgm_file, yaml_file):
    # YAML 파일 읽기
    with open(yaml_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
    
    # PGM 파일 읽기
    pgm_data = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    
    # YAML 파일에서 해상도 및 원점 추출
    resolution = yaml_data['resolution']
    origin = yaml_data['origin']
    
    return pgm_data, resolution, origin

def save_costmap_to_xml(costmap_data, resolution, origin, filename='costmap.xml'):
    # XML 구조 생성
    root = ET.Element('costmap')
    
    # Resolution 요소 추가
    resolution_elem = ET.SubElement(root, 'resolution')
    resolution_elem.text = str(resolution)
    
    # Origin 요소 추가
    origin_elem = ET.SubElement(root, 'origin')
    origin_elem.text = f"{origin[0]} {origin[1]} {origin[2]}"
    
    # Costmap 데이터 추가
    data_elem = ET.SubElement(root, 'data')
    for row in costmap_data:
        row_elem = ET.SubElement(data_elem, 'row')
        row_elem.text = ' '.join(map(str, row))
    
    # XML 파일로 저장
    tree = ET.ElementTree(root)
    tree.write(filename, encoding='utf-8', xml_declaration=True)

def main():
    # 파일 경로
    pgm_file = '/home/adsol/world.map.pgm'
    yaml_file = '/home/adsol/world.map.yaml'
    
    # PGM 및 YAML 파일 읽기
    pgm_data, resolution, origin = read_pgm_and_yaml(pgm_file, yaml_file)
    
    # XML로 저장
    save_costmap_to_xml(pgm_data, resolution, origin, 'costmap.xml')
    print("Costmap saved to 'costmap.xml'")

if __name__ == "__main__":
    main()
