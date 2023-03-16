#!/usr/bin/env python

import rosbag
import sys
from rospy import Time

if len(sys.argv) != 4:
    print("Usage: modify_rosbag_timestamp.py input.bag output.bag time_offset")
    sys.exit(1)

input_bag_file = sys.argv[1]
output_bag_file = sys.argv[2]
time_offset = float(sys.argv[3])

with rosbag.Bag(output_bag_file, 'w') as outbag:
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            t_shifted = Time.from_sec(t.to_sec() + time_offset)
            outbag.write(topic, msg, t_shifted)

print("Modified timestamps in %s and saved as %s with a time offset of %d seconds." % (
    input_bag_file, output_bag_file, time_offset))
