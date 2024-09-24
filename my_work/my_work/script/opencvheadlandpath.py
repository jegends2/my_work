import numpy as np
import cv2
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, MultiPolygon
import json
import os

def load_pgm(pgm_path):
    """Load PGM file and return grayscale image."""
    if not os.path.isfile(pgm_path):
        raise FileNotFoundError(f"PGM file not found: {pgm_path}")
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Failed to load PGM file: {pgm_path}")
    return img

def create_field_mask(img, white_value=254):
    """Create binary mask where white_value is considered as field."""
    _, binary_mask = cv2.threshold(img, white_value-1, 255, cv2.THRESH_BINARY)
    return binary_mask

def find_contours(mask):
    """Find contours in the binary mask."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def create_headland_polygon(contour, offset):
    """Create a headland polygon by offsetting the contour polygon."""
    polygon = Polygon(np.squeeze(contour))
    
    # Create headland polygon with the specified offset
    headland_polygon = polygon.buffer(-offset)
    
    # Check if the headland polygon is valid
    if not headland_polygon.is_valid:
        headland_polygon = headland_polygon.buffer(0)  # Fix any invalid geometry
    return headland_polygon

def convert_coordinates(coords, resolution, origin, img_height):
    """Convert coordinates from image space to RViz space."""
    return [(x * resolution + origin[0], ((img_height - y) * resolution) + origin[1]) for x, y in coords]

def visualize_polygons(polygons, img, resolution, origin):
    """Visualize the polygons along with the original PGM image."""
    fig, ax = plt.subplots()
    
    # Display the PGM image
    ax.imshow(img, cmap='gray', origin='upper', extent=[origin[0], origin[0] + img.shape[1] * resolution, origin[1], origin[1] + img.shape[0] * resolution])
    
    # Plot polygons
    for polygon in polygons:
        if isinstance(polygon, Polygon):
            x, y = polygon.exterior.xy
            ax.plot(x, y, label='Headland Polygon', color='blue')
        elif isinstance(polygon, MultiPolygon):
            for p in polygon.geoms:
                x, y = p.exterior.xy
                ax.plot(x, y, label='Headland Polygon', color='blue')

    ax.set_aspect('equal')
    ax.legend()
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Headland Polygons Visualization')
    plt.show()

def save_polygon_as_json(polygons, filename):
    """Save the polygon coordinates to a JSON file."""
    polygons_data = []
    
    for polygon in polygons:
        if isinstance(polygon, MultiPolygon):
            for poly in polygon.geoms:
                polygons_data.append(list(poly.exterior.coords))
        else:
            polygons_data.append(list(polygon.exterior.coords))
    
    with open(filename, 'w') as f:
        json.dump({'polygons': polygons_data}, f, indent=4)

def extract_polygon_from_contours(contours, resolution, origin, img_height):
    """Extract polygons from contours and convert them to RViz coordinates."""
    polygons = []
    for contour in contours:
        # Create the headland polygon
        headland_polygon = create_headland_polygon(contour, offset=0.35 / resolution)  # Adjust offset to match resolution
        
        # Convert headland polygon coordinates
        if not headland_polygon.is_empty:
            if isinstance(headland_polygon, Polygon):
                converted_coords = convert_coordinates(list(headland_polygon.exterior.coords), resolution, origin, img_height)
                polygons.append(Polygon(converted_coords))
            elif isinstance(headland_polygon, MultiPolygon):
                for poly in headland_polygon.geoms:
                    converted_coords = convert_coordinates(list(poly.exterior.coords), resolution, origin, img_height)
                    polygons.append(Polygon(converted_coords))
    return polygons

def main():
    pgm_path = 'world.map.pgm'  # Replace with your PGM file path
    
    try:
        # Load the PGM image
        img = load_pgm(pgm_path)
        
        # Create a mask for the field
        field_mask = create_field_mask(img)
        
        # Find contours
        contours = find_contours(field_mask)
        
        # Define map parameters from YAML
        resolution = 0.05  # meters per pixel
        origin = (-5.75, -5.06)  # RViz origin coordinates in meters
        
        img_width = img.shape[1]
        img_height = img.shape[0]

        # Extract polygons from contours
        polygons = extract_polygon_from_contours(contours, resolution, origin, img_height)
        
        # Visualize the polygons with the PGM image
        visualize_polygons(polygons, img, resolution, origin)
        
        # Save the polygons to JSON
        if polygons:
            save_polygon_as_json(polygons, 'headland_path.json')
    
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
