import cv2
import numpy as np

def load_pgm_image(pgm_file):
    """Load a PGM image and return as a binary numpy array."""
    image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Image file {pgm_file} not found.")
    return image

def extract_polygon_from_image(image, threshold=0.5):
    """Extract polygon from a binary image."""
    _, binary_image = cv2.threshold(image, int(threshold * 255), 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv2.contourArea)  # Use the largest contour
        contour = np.squeeze(contour)
        return [(point[0], point[1]) for point in contour]
    else:
        raise ValueError("No contours found in the image.")

def main():
    # Load PGM image
    pgm_file = "world.map.pgm"
    image = load_pgm_image(pgm_file)

    # Extract polygon from image
    polygon = extract_polygon_from_image(image)

    # Save polygon to file
    with open("polygon.txt", "w") as f:
        for x, y in polygon:
            f.write(f"{x},{y}\n")

    print("Extracted Polygon Points:", polygon)

if __name__ == '__main__':
    main()
