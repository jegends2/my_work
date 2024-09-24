import numpy as np
from skimage import measure
from shapely.geometry import Polygon
from shapely.ops import unary_union

def extract_field_polygons(costmap, white_value):
    """
    Extract polygons representing the field areas (white areas) from the costmap.
    """
    # 흰색 부분 (필드)만 선택
    field_mask = (costmap == white_value)
    
    # Debug: Check the unique values in the field_mask
    print("Unique values in field_mask:", np.unique(field_mask))

    # 흰색 부분의 윤곽선 추출
    contours = measure.find_contours(field_mask, 0.5)
    
    # Debug: Check the number of contours found
    print("Number of contours found:", len(contours))
    
    if not contours:
        return None

    return [Polygon(p) for p in contours]


def generate_headland(field_polygons, headland_width):
    """
    Generate headland polygons by buffering field polygons.
    """
    headland_polygons = []
    
    for polygon in field_polygons:
        if polygon.is_valid:
            headland = polygon.buffer(headland_width, resolution=16)
            headland_polygons.append(headland)
    
    # Merge headlands
    merged_headlands = unary_union(headland_polygons)
    
    return merged_headlands
