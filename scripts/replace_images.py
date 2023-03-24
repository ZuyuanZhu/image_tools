#!/usr/bin/env python

import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image



bag_file = '/home/zuyuan/Documents/dataset/MH_02_easy.bag'
image_folder = '/media/zuyuan/DATA1TB/kitti/2011_10_03/2011_10_03_drive_0034_sync/image_00/line13-14/'
image_names = []
# Loop through each file in the folder
for filename in os.listdir(image_folder):
    # Check if the file is an image
    if filename.endswith('.jpg') or filename.endswith('.png'):
        # Print the filename
        image_names.append(filename)
image_names.sort()
# Open the input rosbag
bag = rosbag.Bag(bag_file)

# Create a new rosbag for writing
new_bag_file = '/home/zuyuan/Documents/dataset/MH_02_easy_replaced.bag'
new_bag = rosbag.Bag(new_bag_file, 'w')
idx = 0
# Loop through each message in the input rosbag
for topic, msg, t in bag.read_messages():
    # Check if the topic is the one you want to replace the image
    if topic == '/cam0/image_raw':
        if idx == len(image_names):
            break
        # Load the image from the folder
        image_path = image_folder + image_names[idx]
        idx += 1
        print(image_path)
        image = cv2.imread(image_path)

        # Convert the image to a ROS message
        new_msg = Image()
        new_msg.data = image.tostring()
        new_msg.height = image.shape[0]
        new_msg.width = image.shape[1]
        new_msg.encoding = 'bgr8'
        new_msg.step = image.shape[1] * 3

        # Write the new message to the new rosbag
        new_bag.write(topic, new_msg, t)
    else:
        # Write the original message to the new rosbag
        new_bag.write(topic, msg, t)

# Close the input and output rosbags
bag.close()
new_bag.close()
