#!/usr/bin/env python3
"""
Usage:
    python convert_vertical.py input_image.png array_name

This script loads an image, converts it to 1‑bit (monochrome),
and packs the pixels vertically (each byte contains 8 pixels in a column)
into a C‑style array. The output ordering is page‑major:
for each page (group of 8 rows), iterate over columns.
Each byte is built with LSB‐first (bit0 = top pixel in the group),
which matches the output of LCD Assistant’s “Vertical 1 bit per pixel” mode.
If the image still appears off, you can try flipping it vertically.
Requires Pillow: pip install Pillow
"""

import sys
from PIL import Image
import os

def convert_image_to_vertical_c_array(image_path, file_path, array_name):
    # Open the image; uncomment the flip if the image appears upside down.
    im = Image.open(image_path)
    # im = im.transpose(Image.FLIP_TOP_BOTTOM)
    
    # Convert the image to 1-bit mode (pixels become 0 or 255)
    im = im.convert("1")
    width, height = im.size

    byte_list = []
    pages = (height + 7) // 8  # number of vertical pages
    
    # Use page-major order: iterate over pages first, then columns.
    for page in range(pages):
        for x in range(width):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                if y < height:
                    pixel = im.getpixel((x, y))
                    # Use LSB-first: top pixel (lowest y) goes to bit0.
                    if pixel:
                        byte |= (1 << bit)
            byte_list.append(byte)

    # Create the C-style array output.
    output = f"#include \"lcdgfx.h\"\n\n"
    output += f"const uint8_t {array_name}[] PROGMEM = {{\n    "
    hex_values = [f"0x{b:02x}" for b in byte_list]
    for i in range(0, len(hex_values), 12):
        line = ", ".join(hex_values[i:i+12])
        output += line + ",\n    "
    output = output.rstrip(",\n    ") + "\n};\n"
    output += f"// Image dimensions: {width} x {height} (Vertical packing, page-major order, LSB-first)\n"
    return output, array_name

def create_cpp_array_file(contents, path, array_name):
    # Ensure the output directory exists; create it if it doesn't.
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)
    try:
        with open(f"{path}/{array_name}.h", "w") as file:
            file.write(contents)
    except Exception as e:
        print(f"Error writing file: {e}")
    print(f"File Created: {array_name}.h")
    
    
def update_master_header_incremental(master_header_path, new_header_relative_path, asset_name=None):
    """
    Incrementally update the master header file by appending a new include directive
    if it doesn't already exist. new_header_relative_path should be the relative path
    (from the master header file's location) of the new header file.
    """
    header_guard_start = "// Auto-generated master header for all generated frame arrays\n#ifndef GIF_FRAMES_ALL_H\n#define GIF_FRAMES_ALL_H\n\n"
    header_guard_end = "\n#endif // GIF_FRAMES_ALL_H\n"
    # If asset_name is provided, prepend "<asset_name>/headers/"; else prepend "headers/"
    if asset_name is not None:
        prefix = f"{asset_name}/headers/"
        if not new_header_relative_path.startswith(prefix):
            new_header_relative_path = prefix + new_header_relative_path
    else:
        if not new_header_relative_path.startswith("headers/"):
            new_header_relative_path = "headers/" + new_header_relative_path
    print("DEBUG: new_header_relative_path =", new_header_relative_path)
    include_line = f'#include "{new_header_relative_path.replace(os.sep, "/")}"\n'
    
    if os.path.exists(master_header_path):
        with open(master_header_path, "r") as f:
            content = f.read()
    else:
        content = header_guard_start + header_guard_end
    
    if include_line not in content:
        # Insert the new include before the closing guard
        content = content.replace(header_guard_end, include_line + header_guard_end)
        try:
            with open(master_header_path, "w") as f:
                f.write(content)
            print(f"Master header updated incrementally: {master_header_path}")
        except Exception as e:
            print(f"Error updating master header incrementally: {e}")
    else:
        print("Include already exists in master header.")
    

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python convert.py input_image.png array_name (Optional) file_path")
        sys.exit(1)
    image_path = sys.argv[1]
    array_name = sys.argv[2]
    file_path = sys.argv[3] if len(sys.argv) > 3 else "../img/"
    (contents, array_name) = convert_image_to_vertical_c_array(image_path, file_path, array_name)
    create_cpp_array_file(contents=contents, path=file_path, array_name=array_name)
    
    # Incrementally update the master header file
    master_header = os.path.join("../img", "gif_frames_all.h")
    new_header = f"{array_name}.h"
    asset_name = os.path.basename(os.path.dirname(file_path))
    update_master_header_incremental(master_header, new_header, asset_name)