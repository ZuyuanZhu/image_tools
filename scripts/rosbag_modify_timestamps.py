#!/usr/bin/env python

"""
 read a ROS bag file, modify the timestamps for the '/sugv/velodyne_points' topic based on the
 first timestamp from the '/zed2/zed_node_test/left_raw/image_raw_gray' topic, and write the
 modified messages to a new bag file
"""
import rosbag

base = "/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/"
input_bag_path = 'test_2_225_258'
output_bag_path = input_bag_path + "_pointCloud"
input_bag = base + input_bag_path + ".bag"
output_bag = base + output_bag_path + ".bag"

first_camera_ts = None
first_velodyne_ts = None

with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if first_camera_ts is None and topic == '/zed2/zed_node_test/left_raw/image_raw_gray':
            first_camera_ts = msg.header.stamp

        if topic == '/sugv/velodyne_points':
            if first_velodyne_ts is None:
                first_velodyne_ts = msg.header.stamp

            if first_camera_ts is not None:
                time_difference = msg.header.stamp - first_velodyne_ts
                new_timestamp = first_camera_ts + time_difference
                msg.header.stamp = new_timestamp

        outbag.write(topic, msg, t)

print(f"new rosbag saved at: {outbag}")
