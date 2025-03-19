#!/usr/bin/env python3
"""
Script to split a GIF file into separate frame images.
Usage:
    python split_gif.py input.gif [output_directory]

If output_directory is not specified, frames will be saved in a folder named 'frames'.
"""

import os
import sys
import subprocess
from PIL import Image, ImageSequence

def split_gif(gif_path, output_di, frame_list):
    # Ensure the output directory exists and create subdirectories for frames and headers
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    frames_dir = os.path.join(output_dir, "frames")
    headers_dir = os.path.join(output_dir, "headers")
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir, exist_ok=True)
    if not os.path.exists(headers_dir):
        os.makedirs(headers_dir, exist_ok=True)
    
    try:
        im = Image.open(gif_path)
    except IOError:
        print(f"Cannot open GIF file: {gif_path}")
        sys.exit(1)
    
    frame_count = 0
    for frame in ImageSequence.Iterator(im):
        # Convert frame to RGBA (handles transparency if needed)
        frame = frame.convert('RGBA')
        frame_list.append(f"frame_{frame_count:03d}")
        frame_filename = os.path.join(frames_dir, f"frame_{frame_count:03d}.png")
        frame.save(frame_filename, 'PNG')
        print(f"Saved frame {frame_count} to {frame_filename}")

        # Automatically call the convert.py script to convert the saved frame to a C header
        array_name = f"frame_{frame_count:03d}"
        convert_script = os.path.join(os.path.dirname(__file__), "convert.py")
        output_header_dir = headers_dir
        cmd = ["python", convert_script, frame_filename, array_name, output_header_dir]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Conversion error for {frame_filename}:\n{result.stderr}")
        else:
            print(f"Conversion output for {frame_filename}:\n{result.stdout}")

        frame_count += 1
    print(f"Frames List: {frame_list}")
    print(f"Total frames saved: {frame_count}")
    

def generate_frame_list_code(frame_names, array_name="frame_list"):
    """
    Generates a C++ code snippet for an array of frame pointers.
    
    Example output:
    
    const uint8_t* frame_list[] = {
        frame_001,
        frame_002,
        frame_003
    };
    
    Parameters:
      frame_names: List of frame names as strings, e.g. ["frame_001", "frame_002", ...]
      array_name:  Name of the array (default "frame_list")
      
    Returns:
      A string containing the formatted C++ code.
    """
    # Start with the array declaration
    code = f"const uint8_t* {array_name}[] = {{\n    "
    # Join the frame names with commas and line breaks
    code += ",\n    ".join(frame_names)
    code += "\n};\n"
    return code


def append_frames_list(file_path, frame_list):
    """
    Reads the header file, inserts the given string immediately before the first
    occurrence of a line that starts with "#endif", and writes the changes back.
    
    If no "#endif" is found, the string is appended at the end of the file.
    """
    # Read the header file into a list of lines
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Find the line index where "#endif" occurs
    insert_index = None
    for i, line in enumerate(lines):
        if line.strip().startswith("#endif"):
            insert_index = i
            break

    # If no "#endif" is found, set the insert index to the end of the file.
    if insert_index is None:
        insert_index = len(lines)
    
     # Start with the array declaration
    code = f"static const std::list<const uint8_t*> frame_list = {{\n    "
    # Join the frame names with commas and line breaks
    code += ",\n    ".join(frame_list)
    code += "\n};\n"
    # Insert the string (plus a newline) before the #endif line
    lines.insert(insert_index, code)
    
    # Write the updated lines back to the header file
    with open(file_path, 'w') as f:
        f.writelines(''.join(lines))
        

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python split_gif.py input.gif [output_directory]")
        sys.exit(1)
    gif_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "frames"
    global frame_list
    frame_list = []
    split_gif(gif_path, output_dir, frame_list)
    append_frames_list(file_path=f"../img/gif_frames_all.h", frame_list=frame_list)