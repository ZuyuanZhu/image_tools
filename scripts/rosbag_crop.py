#!/usr/bin/env python

import os
from PIL import Image

# keep which area, "upper" or "lower"
area = "lower"

# Source and destination directories
src_dir = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/images/test2'
dst_dir = src_dir + "_" + area + "_half"

if not os.path.exists(dst_dir):
    os.makedirs(dst_dir)

# Iterate through all files in source directory
for filename in os.listdir(src_dir):
    if filename.endswith('.jpg') or filename.endswith('.png'):  # Add/modify file extensions as needed
        # Open image file
        img = Image.open(os.path.join(src_dir, filename))
        width, height = img.size

        if area == "upper":
            # Crop the upper half of the image
            img_cropped = img.crop((0, 0, width, height // 2))
        elif area == "lower":
            # Crop the lower half of the image
            img_cropped = img.crop((0, height // 2, width, height))

        # Save cropped image to destination directory
        img_cropped.save(os.path.join(dst_dir, filename))

print("Image cropping completed!")
