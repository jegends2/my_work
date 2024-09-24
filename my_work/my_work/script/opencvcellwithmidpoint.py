import numpy as np
import cv2
import matplotlib.pyplot as plt
import json
import yaml
import os

def load_pgm(pgm_path):
    """Load PGM file and return grayscale image."""
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    return img

def load_yaml(yaml_path):
    """Load YAML file containing map resolution and origin."""
    with open(yaml_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    resolution = yaml_data['resolution']
    origin = tuple(yaml_data['origin'])
    return resolution, origin

def create_field_mask(img, white_value=254):
    """Create binary mask where white_value is considered as field."""
    _, binary_mask = cv2.threshold(img, white_value-1, 255, cv2.THRESH_BINARY)
    return binary_mask

def visualize_with_costmap(cells, midpoints, img, resolution, origin):
    """Visualize the costmap with cells and their midpoints."""
    fig, ax = plt.subplots()
    
    # Display the original PGM image (costmap)
    ax.imshow(img, cmap='gray', origin='upper', extent=[origin[0], origin[0] + img.shape[1] * resolution, origin[1], origin[1] + img.shape[0] * resolution])
    
    # Plot cells
    for cell in cells:
        x, y, w, h = cell
        rect = plt.Rectangle((x, y), w, h, linewidth=1, edgecolor='blue', facecolor='none')
        ax.add_patch(rect)

    # Plot midpoints
    for (mx, my) in midpoints:
        ax.plot(mx, my, 'ro', markersize=5)
    
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Costmap with Cells and Midpoints')
    plt.show()

def save_cells_as_json(cells, midpoints, filename):
    """Save the cell and midpoint coordinates to a JSON file."""
    cells_data = [{'x': x, 'y': y, 'width': w, 'height': h} for (x, y, w, h) in cells]
    midpoints_data = [{'x': mx, 'y': my} for (mx, my) in midpoints]
    
    data = {'cells': cells_data, 'midpoints': midpoints_data}
    
    try:
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"File {filename} saved successfully.")
    except IOError as e:
        print(f"IOError while saving file {filename}: {e}")
    except Exception as e:
        print(f"Unexpected error while saving file {filename}: {e}")

def generate_cells_from_mask(mask, img, resolution, origin):
    """Generate cells from the binary mask and calculate their midpoints."""
    height, width = mask.shape
    cell_size = int(0.5 / resolution)  # Size of each cell in pixels

    cells = []
    midpoints = []

    # Debugging: Check the unique values in the mask
    unique_values = np.unique(mask)
    print(f"Unique values in mask: {unique_values}")

    for y in range(0, height, cell_size):
        for x in range(0, width, cell_size):
            cell = mask[y:y+cell_size, x:x+cell_size]
            if np.all(cell == 255):  # Adjust value based on the actual white value in your image
                cells.append((x * resolution + origin[0], (height - y - cell_size) * resolution + origin[1], 
                              cell_size * resolution, cell_size * resolution))
                midpoints.append((x * resolution + origin[0] + (cell_size * resolution / 2), 
                                  (height - y - cell_size / 2) * resolution + origin[1]))
    
    return cells, midpoints

def main():
    pgm_path = 'world.map.pgm'  # Replace with your PGM file path
    yaml_path = 'world.map.yaml'  # Replace with your YAML file path

    # Load image and parameters
    img = load_pgm(pgm_path)
    resolution, origin = load_yaml(yaml_path)

    # Create mask for white areas
    field_mask = create_field_mask(img)

    # Generate cells and their midpoints
    cells, midpoints = generate_cells_from_mask(field_mask, img, resolution, origin)

    # Visualize the costmap with cells and midpoints
    visualize_with_costmap(cells, midpoints, img, resolution, origin)

    # Save cells and midpoints to JSON
    save_cells_as_json(cells, midpoints, 'cell_midpoint.json')

if __name__ == '__main__':
    main()
