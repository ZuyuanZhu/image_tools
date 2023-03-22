#!/usr/bin/env python

import rosbag
import rospy

bag_file = '/home/zuyuan/rasberry_ws/src/cslam_datasets/MH_01_easy.bag'
new_bag_file = '/home/zuyuan/rasberry_ws/src/cslam_datasets/MH_01_easy_split_100_200.bag'
start_num = 100
end_num = 200

# Open the input rosbag
bag = rosbag.Bag(bag_file)

# Create a new rosbag for writing
new_bag = rosbag.Bag(new_bag_file, 'w')

fisrt_msg = True
init_seq = 0
# Loop through each message in the input rosbag
for topic, msg, t in bag.read_messages():
    # Check if the topic is the one you want to keep
    if topic == '/cam0/image_raw':
        # if fisrt_msg:
        #     init_seq = msg.header.seq
        #     print("init_seq: %d" % init_seq)
        #     fisrt_msg = False
        # Check if the sequence number is between 100 and 200
        if start_num <= msg.header.seq <= end_num:
            # Write the message to the new rosbag
            new_bag.write(topic, msg, t)
        elif msg.header.seq > end_num:
            break
    else:
        # Write the original message to the new rosbag
        new_bag.write(topic, msg, t)

# Close the input and output rosbags
bag.close()
new_bag.close()
