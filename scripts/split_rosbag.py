#!/usr/bin/env python
import rosbag
import rospy

bag_file = '/home/zuyuan/rasberry_ws/src/cslam_datasets/MH_01_easy.bag'
new_bag_file = '/home/zuyuan/rasberry_ws/src/cslam_datasets/MH_01_easy_321.bag'
image_count = 321
# Open the input rosbag
bag = rosbag.Bag(bag_file)

# Create a new rosbag for writing
new_bag = rosbag.Bag(new_bag_file, 'w')

count = 0
for topic, msg, t in bag.read_messages():
    # Check if the message is a reset and skip it
    if topic == '/cam0/image_raw':
        # Write the message to the new rosbag
        new_bag.write(topic, msg, t)

        # Increment the message count
        count += 1

        # Stop writing messages after the first 300
        if count >= image_count:
            break


# Close the input and output rosbags
bag.close()
new_bag.close()
