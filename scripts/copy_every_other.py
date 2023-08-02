#!/usr/bin/env python3

import os
import shutil

src_dir = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/images/test2_lower_half'
dst_dir = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/images/test2_lower_half_every_other'  # replace with the path to your destination directory
image_extension = '.jpg'  # replace with your image extension

# Ensure the destination directory exists
if not os.path.exists(dst_dir):
    os.makedirs(dst_dir)

# Get a sorted list of all image filenames in the source directory
image_filenames = sorted([f for f in os.listdir(src_dir) if f.endswith(image_extension)])

# Copy every other image
for i in range(0, len(image_filenames), 2):
    src_path = os.path.join(src_dir, image_filenames[i])
    dst_path = os.path.join(dst_dir, image_filenames[i])
    shutil.copy2(src_path, dst_path)

print('Done.')
