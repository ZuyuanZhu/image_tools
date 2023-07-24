#!/usr/bin/env python

import rosbag
from rosbag.bag import Bag

base_path = "/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/"
input_bag = "test_2_225_258"
input_bag_path = base_path + input_bag + ".bag"
output_bag_path = base_path + input_bag + "_select.bag"

selected_topics = ["/zed2/zed_node_test/odom", "/zed2/zed_node_test/left_raw/image_raw_gray"]

with Bag(output_bag_path, 'w') as output_bag:
    print("Reading rosbag msgs...")
    for topic, msg, t in Bag(input_bag_path).read_messages():
        if topic in selected_topics:
            output_bag.write(topic, msg, t)

print("Filtered rosbag saved at {}".format(output_bag_path))
