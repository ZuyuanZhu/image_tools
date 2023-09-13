#!/usr/bin/env python

import rosbag
from rosbag.bag import Bag

base_path = "/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/"
input_bag = "test_2_2023-07-04-10-13-18"
input_bag_path = base_path + input_bag + ".bag"
output_bag_path = base_path + input_bag + "_select_tf.bag"

selected_topics = [
    "/zed2/zed_node_test/odom",
    "/zed2/zed_node_test/left_raw/image_raw_gray",
    "/sugv/velodyne_points",
    "/tf",
    "/tf_static"
]

connection_headers = {}
tf_static_msgs = []

with rosbag.Bag(input_bag_path, 'r') as inbag:
    # Collect first two /tf_static messages
    tf_static_counter = 0
    for topic, msg, t in inbag.read_messages(topics=["/tf_static"]):
        if tf_static_counter < 2:
            tf_static_msgs.append((topic, msg, t))
            tf_static_counter += 1
        else:
            break

with rosbag.Bag(output_bag_path, 'w') as outbag:
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic in selected_topics:
                # If the current topic is /tf_static, add the stored messages first
                if topic == "/tf_static":
                    outbag.write(topic, msg, t, connection_header=connection_headers.get(topic, None))
                    for tf_topic, tf_msg, tf_t in tf_static_msgs:
                        outbag.write(tf_topic, tf_msg, t, connection_header=connection_headers.get(tf_topic, None))

                if topic not in connection_headers:
                    # Build connection header for the first occurrence of each topic
                    connection_headers[topic] = {
                        "topic": topic,
                        "type": msg._type,
                        "md5sum": msg._md5sum,
                        "message_definition": msg._full_text,
                        "callerid": "select_topics_script",
                        "latching": "0"
                    }
                # Now write the message with the connection header
                outbag.write(topic, msg, t, connection_header=connection_headers[topic])

print("Filtered rosbag saved at {}".format(output_bag_path))
