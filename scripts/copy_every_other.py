#!/usr/bin/env python3

import os
import shutil

src_dir = '/home/zuyuan/dataset/jackal/jackal_zed_small_Fri_1_0'
dst_dir = '/home/zuyuan/dataset/jackal/jackal_zed_small_Fri_1_0_every_other'  # replace with the path to your destination directory
image_extension = '.jpg'  # replace with your image extension

# Ensure the destination directory exists
os.makedirs(dst_dir, exist_ok=True)

# Get a sorted list of all image filenames in the source directory
image_filenames = sorted([f for f in os.listdir(src_dir) if f.endswith(image_extension)])

# Copy every other image
for i in range(0, len(image_filenames), 2):
    src_path = os.path.join(src_dir, image_filenames[i])
    dst_path = os.path.join(dst_dir, image_filenames[i])
    shutil.copy2(src_path, dst_path)

print('Done.')
