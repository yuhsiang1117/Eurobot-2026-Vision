import cv2
import cv2.aruco as aruco
import numpy as np

# --- Configuration ---
# Board size
squares_x = 7
squares_y = 5

# Square size (in meters)
square_length = 0.04

# Marker size (in meters)
marker_length = 0.02

# ArUco dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Output file name
output_file = "charuco_board.png"

# --- Generation ---
# Create the board
board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)

# Create an image from the board
board_image = board.generateImage((600*squares_x, 600*squares_y))

# Save the image
cv2.imwrite(output_file, board_image)

print(f"Charuco board saved to {output_file}")
print("\nBoard configuration:")
print(f"  - Squares X: {squares_x}")
print(f"  - Squares Y: {squares_y}")
print(f"  - Square length: {square_length} m")
print(f"  - Marker length: {marker_length} m")
print(f"  - Dictionary: DICT_4X4_100")

print("\nPrinting instructions:")
print("1. Open the generated image 'charuco_board.png'.")
print("2. Print the image at its original size. Use a high-quality printer if possible.")
print("3. Attach the printed board to a rigid, flat surface.")
