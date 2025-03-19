#!/usr/bin/env python3
"""
Usage:
    python convert.py image_path array_name [--path=PATH]

This script loads an image, converts it to 1‑bit (monochrome),
and packs the pixels vertically into a C‑style array.
It generates a header file containing the C array.

Options:
    --path PATH  Directory to save the generated header file [default: ../img/headers]
"""

import os
import argparse
from PIL import Image

def convert_image_to_vertical_c_array(image_path, array_name):
    im = Image.open(image_path)
    im = im.convert("1")  # Convert to 1-bit monochrome
    width, height = im.size

    byte_list = []
    pages = (height + 7) // 8  # Number of vertical pages
    
    # Page-major order: iterate over pages then columns.
    for page in range(pages):
        for x in range(width):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                if y < height:
                    pixel = im.getpixel((x, y))
                    if pixel:
                        byte |= (1 << bit)  # LSB-first: top pixel -> bit0
            byte_list.append(byte)

    output = '#include "lcdgfx.h"\n\n'
    output += f"const uint8_t {array_name}[] PROGMEM = {{\n    "
    hex_values = [f"0x{b:02x}" for b in byte_list]
    for i in range(0, len(hex_values), 12):
        line = ", ".join(hex_values[i:i+12])
        output += line + ",\n    "
    output = output.rstrip(",\n    ") + "\n};\n"
    output += f"// Image dimensions: {width} x {height} (Vertical packing, page-major order, LSB-first)\n"
    return output

def create_cpp_array_file(contents, path, array_name):
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)
    file_path = os.path.join(path, f"{array_name}.h")
    with open(file_path, "w") as file:
        file.write(contents)
    print(f"Header file created: {file_path}")

def main():
    parser = argparse.ArgumentParser(description="Convert image to vertical C array and generate header file.")
    parser.add_argument("image_path", help="Path to input image file.")
    parser.add_argument("array_name", help="Name of the C array.")
    parser.add_argument("--path", "-p", default="../img/headers", help="Output directory for header file.")
    args = parser.parse_args()

    contents = convert_image_to_vertical_c_array(args.image_path, args.array_name)
    create_cpp_array_file(contents, args.path, args.array_name)

if __name__ == "__main__":
    main()