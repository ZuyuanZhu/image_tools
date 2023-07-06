#!/usr/bin/env python
import rosbag

bag_path = '/media/zuyuan/DATA1TB/Jackal/'
# Name of the original bag file
input_bag = 'jackal_zed_small_Fri_1_0_agent_0_frame_id_odom.bag'
input_bag = bag_path + input_bag
# Name for the new bag file
output_bag = 'jackal_zed_small_Fri_1_0_agent_0_frame_id_odom_filtered.bag'
output_bag = bag_path + output_bag

# List of undesired image frame ids
undesired_ids = ['0000000884', '0000001683']

# Time tolerance in seconds (to filter messages around the same time as undesired images)
tolerance = 0.05

# Dictionary to store the undesired timestamps
undesired_timestamps = {}

with rosbag.Bag(input_bag) as inbag:
    # First pass to get the undesired timestamps
    for topic, msg, t in inbag.read_messages():
        if topic == '/zed/zed_node/left/image_rect_color' and msg.header.frame_id in undesired_ids:
            undesired_timestamps[t.to_sec()] = True

with rosbag.Bag(output_bag, 'w') as outbag:
    # Second pass to filter out undesired messages
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        # Check if the timestamp is within tolerance of any undesired timestamp
        for undesired_t in undesired_timestamps:
            if abs(t.to_sec() - undesired_t) < tolerance:
                break
        else:
            # If not within tolerance of any undesired timestamp, copy the message
            outbag.write(topic, msg, t)
