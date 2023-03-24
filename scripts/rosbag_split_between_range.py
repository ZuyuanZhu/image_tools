#!/usr/bin/env python

import rosbag
import rospy

# bag_file = '/media/zuyuan/DATA1TB/kitti/kitti_2011_10_03_drive_0034_synced.bag'
# new_bag_file = '/media/zuyuan/DATA1TB/kitti/kitti_2011_10_03_drive_0034_synced_1630_1950.bag'
# bag_file = '/home/zuyuan/Documents/dataset/kitti/raw/2011_10_03_drive_0034_sync_loop5-6.bag'
new_bag_file = '/home/zuyuan/Documents/dataset/kitti/raw/2011_10_03_drive_0034_sync_loop5-6_select.bag'
# bag_file = '/home/zuyuan/Documents/dataset/MH_01_easy.bag'
bag_file = '/media/zuyuan/DATA1TB/kitti/kitti_splited/2011_10_03/output/line5-6/bags.bag'

start_num = 1630
end_num = 1950

# Open the input rosbag
bag = rosbag.Bag(bag_file)

# Create a new rosbag for writing
new_bag = rosbag.Bag(new_bag_file, 'w')

fisrt_msg = True
init_seq = 0
# Loop through each message in the input rosbag
for topic, msg, t in bag.read_messages():
    print(t)
    print("Read msg seq: %d" % msg.header.seq)
    # Check if the topic is the one you want to keep
    if topic == '/kitti/camera_gray_left/image_raw':
        # print("topic: %s" % topic)
        # if fisrt_msg:
        #     init_seq = msg.header.seq
        #     print("init_seq: %d" % init_seq)
        #     fisrt_msg = False
        # Check if the sequence number is between 100 and 200
        if start_num <= msg.header.seq <= end_num:
            # Write the message to the new rosbag
            print("Write msg seq: %d" % msg.header.seq)
            new_bag.write(topic, msg, t)
        elif msg.header.seq > end_num:
            break
    else:
        # Write the original message to the new rosbag
        pass
        # new_bag.write(topic, msg, t)

# Close the input and output rosbags
bag.close()
new_bag.close()
