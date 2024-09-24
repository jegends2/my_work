import yaml
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from skimage import morphology

# Specify the path to the YAML file directly
YAML_PATH = '/home/adsol/world.map.yaml'

def load_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def shrink_map(image, shrink_distance, resolution):
    """Shrink the black areas of the map by a given distance."""
    image_array = np.array(image)

    # Create a binary mask where black areas are True
    binary_mask = image_array < 128

    # Create a smaller footprint for dilation
    footprint_radius = shrink_distance / resolution
    footprint = morphology.disk(footprint_radius)
    
    # Apply dilation to expand the black areas
    dilated_mask = morphology.binary_dilation(binary_mask, footprint=footprint)

    # Create a new image with the shrunk black areas
    shrunk_image_array = np.where(dilated_mask, 0, image_array)
    
    # Optional: Post-process to clean up small artifacts
    cleaned_image_array = morphology.remove_small_objects(shrunk_image_array > 128, min_size=50)
    
    return Image.fromarray(cleaned_image_array)

def main(args=None):
    # Load the YAML data
    data = load_yaml(YAML_PATH)
    resolution = data['resolution']
    image_path = data['image']
    
    # Open the image
    original_image = Image.open(image_path).convert('L')
    
    # Shrink the map
    shrink_distance = 0.3  # Distance in meters
    shrunk_image = shrink_map(original_image, shrink_distance, resolution)
    
    # Save the shrunk image
    shrunk_image.save('shrunk_map.png')
    
    # Display both maps
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    axs[0].imshow(original_image, cmap='gray')
    axs[0].set_title('Original Map')
    axs[0].axis('off')
    
    axs[1].imshow(shrunk_image, cmap='gray')
    axs[1].set_title('Shrunk Map')
    axs[1].axis('off')
    
    plt.show()

if __name__ == '__main__':
    main()
