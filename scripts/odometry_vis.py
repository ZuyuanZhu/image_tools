#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt

# Path to your rosbag
bag_path = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/test_2_2023-07-04-10-13-18.bag'

x_data = []
y_data = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/sugv/odometry/filtered']):
        x_data.append(msg.pose.pose.position.x)
        y_data.append(msg.pose.pose.position.y)

plt.figure()
plt.plot(x_data, y_data)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Odometry Data')
plt.show()
