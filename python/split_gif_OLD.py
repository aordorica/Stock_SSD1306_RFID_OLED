#!/usr/bin/env python3
"""
Script to split a GIF file into separate frame images and update the master header.
Usage:
    python split_gif.py [--overwrite] [--confirm] input.gif [output_directory]

If output_directory is not specified, the default is the GIF filename (without extension).

Behavior:
  - If the header/frames parent directory exists:
       * With the --overwrite flag (or -o), the script prompts (or automatically confirms if --confirm is set) and then
         skips full splitting—updating only the master header file using the existing header files.
  - If the parent directory does not exist:
       * The script creates the directory, splits the GIF (saving frame images and generating individual header files),
         and then generates the master header file.
"""

import os
import sys
import subprocess
from PIL import Image, ImageSequence
import shutil

# Path to the master header file (relative to ../img)
MASTER_HEADER_PATH = "../img/gif_frames_all.h"

def split_gif(gif_path, output_dir, frame_list):
    # Adjust output_dir if provided as a filename.
    ext = os.path.splitext(output_dir)[1].lower()
    if ext in ['.gif', '.png', '.jpg', '.jpeg']:
        output_dir = os.path.join(os.path.dirname(output_dir), os.path.splitext(os.path.basename(output_dir))[0])
    
    # Ensure output directory exists.
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    # Create subdirectories for frames and headers.
    frames_dir = os.path.join(output_dir, "frames")
    headers_dir = os.path.join(output_dir, "headers")
    os.makedirs(frames_dir, exist_ok=True)
    os.makedirs(headers_dir, exist_ok=True)
    
    try:
        im = Image.open(gif_path)
    except IOError:
        print(f"Cannot open GIF file: {gif_path}")
        sys.exit(1)
    
    frame_count = 0
    for frame in ImageSequence.Iterator(im):
        frame = frame.convert('RGBA')  # Convert to RGBA to handle transparency
        frame_name = f"frame_{frame_count:03d}"
        frame_list.append(frame_name)
        
        # Save the frame image.
        frame_filename = os.path.join(frames_dir, f"{frame_name}.png")
        frame.save(frame_filename, 'PNG')
        print(f"Saved frame {frame_count} to {frame_filename}")
        
        # Call convert.py to convert the saved frame into a header file.
        convert_script = os.path.join(os.path.dirname(__file__), "convert.py")
        cmd = ["python", convert_script, frame_filename, frame_name, "-p", headers_dir]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Conversion error for {frame_filename}:\n{result.stderr}")
        else:
            print(f"Conversion output for {frame_filename}:\n{result.stdout}")
        frame_count += 1

    print(f"Total frames processed: {frame_count}")

def get_existing_frame_list(headers_dir):
    frames = []
    if not os.path.exists(headers_dir):
        return frames
    for fname in os.listdir(headers_dir):
        if fname.startswith("frame_") and fname.endswith(".h"):
            frame_name = os.path.splitext(fname)[0]
            frames.append(frame_name)
    frames.sort()
    return frames

def update_master_header_new(frame_list, output_dir, master_header_path):
    # The asset directory name (e.g., "Scary_cat_frames") is assumed to be the basename of output_dir.
    asset_name = os.path.basename(output_dir)
    header_lines = []
    header_lines.append("// Auto-generated master header for all generated frame arrays")
    header_lines.append("#include <list>")
    header_lines.append("")
    header_lines.append(f"#define {str.upper(asset_name)}")
    header_lines.append(f"#ifdef {str.upper(asset_name)}")
    header_lines.append("")
    for frame in frame_list:
        header_lines.append(f'#include "{asset_name}/headers/{frame}.h"')
    header_lines.append("")
    header_lines.append("static const std::list<const uint8_t *> frame_list = {")
    for frame in frame_list:
        header_lines.append(f"    {frame},")
    header_lines.append("};")
    header_lines.append("")
    header_lines.append("#endif")
    master_content = "\n".join(header_lines) + "\n"
    with open(master_header_path, "w") as f:
        f.write(master_content)
    print(f"Master header file updated: {master_header_path}")

def main():
    args = sys.argv[1:]
    overwrite_flag = False
    confirm_flag = False

    # Process flags: --overwrite (or -o) and -c/--confirm (as boolean flags).
    if "--overwrite" in args or "-o" in args:
        overwrite_flag = True
        args = [arg for arg in args if arg not in ("--overwrite", "-o")]
    if "--confirm" in args or "-c" in args:
        confirm_flag = True
        args = [arg for arg in args if arg not in ("--confirm", "-c")]

    if len(args) < 1:
        print("Usage: python split_gif.py [--overwrite] [--confirm] input.gif [output_directory]")
        sys.exit(1)
    
    gif_path = args[0]
    output_dir = args[1] if len(args) > 1 else os.path.splitext(os.path.basename(gif_path))[0]
    
    # Compute the headers directory.
    headers_dir = os.path.join(output_dir, "headers")
    
    # Change working directory to ../img.
    os.chdir("../img")
    full_output_dir = os.path.abspath(output_dir)
    
    if os.path.isdir(output_dir):
        dir_name = os.path.basename(full_output_dir)
        print(f"Frame file directory 'img/{dir_name}' found!")
        if overwrite_flag:
            if confirm_flag:
                answer = "y"
            else:
                answer = input(f"Would you like to update gif_frames_all.h's master list?\n\033[1mUpdate (Y/N): \033[0m")
            if answer.lower() != 'y':
                print("Aborting overwrite.")
                sys.exit(0)
            # Use existing header files to build frame list and update master header.
            frame_list = get_existing_frame_list(headers_dir)
            if not frame_list:
                print(f"No header files found in '{headers_dir}'. Running full splitting process.")
                frame_list = []
                split_gif(gif_path, output_dir, frame_list)
            update_master_header_new(frame_list, output_dir, MASTER_HEADER_PATH)
            sys.exit(0)
        else:
            if confirm_flag:
                answer = "y"
            else:
                answer = input(f"Would you like to overwrite it and re-split? \033[1m(y/n): \033[0m")
            if answer.lower() != 'y':
                if confirm_flag:
                    update_answer = "y"
                else:
                    update_answer = input("Update master header file only from existing headers? \033[1m(y/n): \033[0m")
                if update_answer.lower() == 'y':
                    frame_list = get_existing_frame_list(headers_dir)
                    update_master_header_new(frame_list, output_dir, MASTER_HEADER_PATH)
                    sys.exit(0)
                else:
                    print("Aborting splitting.")
                    sys.exit(0)
            shutil.rmtree(output_dir)
            os.makedirs(output_dir, exist_ok=True)
    
    # If output directory does not exist (or after deletion), run full splitting.
    frame_list = []
    split_gif(gif_path, output_dir, frame_list)
    update_master_header_new(frame_list, output_dir, MASTER_HEADER_PATH)

if __name__ == "__main__":
    main()