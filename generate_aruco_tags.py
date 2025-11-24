import cv2
import numpy as np
import os

# --- Parameters ---
# Dictionary to use
# This MUST match the dictionary in your C++ code: DICT_4X4_100
dictionary_name = cv2.aruco.DICT_4X4_100
dictionary = cv2.aruco.getPredefinedDictionary(dictionary_name)

# Marker IDs to generate, as specified in your C++ code
marker_ids = [20, 21, 22, 23]

# Size of the generated image in pixels
# A larger size gives you a higher resolution image to print
image_size = 500

# Create a directory to save the markers
output_dir = "aruco_tags"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

print(f"Generating markers for dictionary {dictionary_name}...")

# --- Generation Loop ---
for marker_id in marker_ids:
    # Create an empty image
    marker_image = np.zeros((image_size, image_size), dtype=np.uint8)

    # Draw the marker
    cv2.aruco.drawMarker(dictionary, marker_id, image_size, marker_image, 1)

    # Add a white border for better detection
    bordered_image = cv2.copyMakeBorder(
        marker_image,
        int(image_size * 0.1),
        int(image_size * 0.1),
        int(image_size * 0.1),
        int(image_size * 0.1),
        cv2.BORDER_CONSTANT,
        value=[255]
    )

    # Save the image
    file_name = os.path.join(output_dir, f"marker_{marker_id}.png")
    cv2.imwrite(file_name, bordered_image)
    print(f"- Saved {file_name}")

print("\nGeneration complete.")
print("You can now print these images.")
print("IMPORTANT: When printing, ensure the black square of the marker is exactly 10cm x 10cm.")
