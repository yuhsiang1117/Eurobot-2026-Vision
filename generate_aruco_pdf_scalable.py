import cv2
import numpy as np
import os
import argparse
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas

# --- Default Parameters ---
# These can be overridden by command-line arguments
DEFAULT_DICTIONARY = "DICT_4X4_100"
DEFAULT_MARKER_SIZE_CM = 7.0
DEFAULT_SPACING_CM = 1.0 # Space between markers
DEFAULT_DPI = 300

def get_aruco_dictionary(dict_name_str):
    """Gets the ArUco dictionary object from its string name."""
    try:
        dict_name_enum = cv2.aruco.__getattribute__(dict_name_str.upper())
        return cv2.aruco.getPredefinedDictionary(dict_name_enum)
    except AttributeError:
        print(f"Error: ArUco dictionary '{dict_name_str}' not found.")
        exit(1)

def generate_marker_image(marker_id, dictionary, marker_size_cm, dpi, temp_dir):
    """Generates a single high-resolution PNG file for an ArUco marker."""
    marker_size_inches = marker_size_cm / 2.54
    image_size_pixels = int(marker_size_inches * dpi)

    # Create a black image
    marker_image = np.zeros((image_size_pixels, image_size_pixels), dtype=np.uint8)
    # Draw the ArUco marker (which is white on a black background)
    cv2.aruco.drawMarker(dictionary, marker_id, image_size_pixels, marker_image, 1)
    
    # Save the image
    file_path = os.path.join(temp_dir, f"marker_{marker_id}.png")
    cv2.imwrite(file_path, marker_image)
    return file_path

def create_pdf(marker_ids, dictionary, marker_size_cm, spacing_cm, dpi, pdf_filename):
    """Creates an A4 PDF with the markers arranged automatically in a grid."""
    temp_dir = "temp_aruco_images_scalable"
    if not os.path.exists(temp_dir):
        os.makedirs(temp_dir)

    print(f"Generating {len(marker_ids)} markers...")
    image_paths = [generate_marker_image(mid, dictionary, marker_size_cm, dpi, temp_dir) for mid in marker_ids]
    
    print(f"\nCreating PDF: {pdf_filename}...")
    c = canvas.Canvas(pdf_filename, pagesize=A4)
    page_width, page_height = A4
    
    marker_size_pt = marker_size_cm * cm
    spacing_pt = spacing_cm * cm
    
    # --- Calculate Grid Layout ---
    page_margin = 1.0 * cm # 1cm margin around the page
    drawable_width = page_width - (2 * page_margin)
    drawable_height = page_height - (2 * page_margin)
    
    markers_per_row = int((drawable_width + spacing_pt) / (marker_size_pt + spacing_pt))
    markers_per_col = int((drawable_height + spacing_pt) / (marker_size_pt + spacing_pt))
    markers_per_page = markers_per_row * markers_per_col

    if markers_per_page == 0:
        print(f"Error: Marker size ({marker_size_cm}cm) is too large to fit on an A4 page.")
        return

    col, row = 0, 0
    for i, (marker_id, img_path) in enumerate(zip(marker_ids, image_paths)):
        if i > 0 and i % markers_per_page == 0:
            c.showPage() # Start a new page
            col, row = 0, 0
        
        # Calculate position (from top-left corner)
        x = page_margin + col * (marker_size_pt + spacing_pt)
        y = page_height - page_margin - marker_size_pt - row * (marker_size_pt + spacing_pt)
        
        c.drawImage(img_path, x, y, width=marker_size_pt, height=marker_size_pt)
        
        # Add a text label
        label = f"ID: {marker_id}"
        text_width = c.stringWidth(label)
        c.drawString(x + (marker_size_pt - text_width) / 2, y - 20, label)

        # Update grid position
        col += 1
        if col >= markers_per_row:
            col = 0
            row += 1

    c.save()
    print("PDF saved successfully.")

    # --- Cleanup ---
    print("\nCleaning up temporary files...")
    for path in image_paths:
        os.remove(path)
    os.rmdir(temp_dir)
    print("Cleanup complete.")

def main():
    parser = argparse.ArgumentParser(
        description="Generate a printable A4 PDF of ArUco markers with a flexible grid layout.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # --- Marker ID Arguments ---
    id_group = parser.add_mutually_exclusive_group(required=True)
    id_group.add_argument(
        '--ids', 
        metavar='ID', 
        type=int, 
        nargs='+', 
        help="A list of specific marker IDs to generate (e.g., --ids 5 10 15)"
    )
    id_group.add_argument(
        '--id-range', 
        metavar='START', 
        type=int, 
        nargs=2, 
        help="A range of marker IDs [START, END] (inclusive) to generate (e.g., --id-range 20 30)"
    )
    
    # --- Customization Arguments ---
    parser.add_argument(
        '-s', '--size', 
        type=float, 
        default=DEFAULT_MARKER_SIZE_CM, 
        help=f"The size of the marker's black square in centimeters. Default: {DEFAULT_MARKER_SIZE_CM} cm"
    )
    parser.add_argument(
        '-d', '--dict', 
        type=str, 
        default=DEFAULT_DICTIONARY,
        help=f"The ArUco dictionary to use. Default: {DEFAULT_DICTIONARY}\n(e.g., DICT_5X5_250, DICT_ARUCO_ORIGINAL)"
    )
    parser.add_argument(
        '--spacing',
        type=float,
        default=DEFAULT_SPACING_CM,
        help=f"The spacing between markers on the PDF. Default: {DEFAULT_SPACING_CM} cm"
    )
    parser.add_argument(
        '-o', '--output',
        type=str,
        default="aruco_tags_printable.pdf",
        help="The output PDF filename. Default: aruco_tags_printable.pdf"
    )
    
    args = parser.parse_args()
    
    # --- Process arguments ---
    if args.id_range:
        marker_ids = list(range(args.id_range[0], args.id_range[1] + 1))
    else:
        marker_ids = args.ids
        
    dictionary = get_aruco_dictionary(args.dict)
    
    create_pdf(marker_ids, dictionary, args.size, args.spacing, DEFAULT_DPI, args.output)

if __name__ == "__main__":
    main()
