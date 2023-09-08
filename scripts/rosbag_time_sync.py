#!/usr/bin/env python

import rosbag
from std_msgs.msg import Header


def get_initial_timestamps(input_bag, topics):
    topic_first_timestamps = {}
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic in topics and topic not in topic_first_timestamps:
                if hasattr(msg, "header") and hasattr(msg.header, 'stamp'):
                    topic_first_timestamps[topic] = msg.header.stamp
            if len(topic_first_timestamps) == len(topics):
                break
    return topic_first_timestamps


def adjust_timestamp(input_bag, output_bag, initial_timestamps):
    earliest_time = min(initial_timestamps.values())

    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                # Only adjust messages with a header
                if hasattr(msg, "header") and msg.header.stamp.secs != 0:
                    msg.header.stamp = earliest_time + (msg.header.stamp - initial_timestamps[topic])
                outbag.write(topic, msg, t)


if __name__ == '__main__':
    # parser = ArgumentParser(
    #     description="Synchronize a rosbag using the start time of a reference topic or the bag's start time.")
    # parser.add_argument('input_bag', help='Input bag file')
    # parser.add_argument('output_bag', help='Output bag file')
    # parser.add_argument('--reference_topic', help='Reference topic to align all other topics with', default=None)

    # args = parser.parse_args()
    base = "/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/"
    input_bag_path = 'test_2_2023-07-04-10-13-18'
    output_bag_path = input_bag_path + "_synctime"
    input_bag = base + input_bag_path + ".bag"
    output_bag = base + output_bag_path + ".bag"
    reference_topic = None

    # List of topics in your rosbag that have a header and you wish to adjust
    topics = ['/rosout', '/sugv/enu_pose', '/rosout_agg', '/sugv/imu/data', '/sugv/imu/time_ref',
              '/sugv/emlid_rtk_rover/tcpfix', '/sugv/husky_velocity_controller/odom', '/sugv/joint_states',
              '/zed2/zed_node_test/imu/data', '/sugv/odometry/filtered',
              '/zed2/zed_node_test/imu/mag', '/zed2/zed_node_test/imu/data_raw',
              '/zed2/zed_node_test/depth/camera_info',
              '/sugv/joy_teleop/joy', '/sugv/velodyne_points', '/zed2/zed_node_test/left/camera_info',
              '/zed2/zed_node_test/left_raw/camera_info', '/zed2/zed_node_test/right_raw/camera_info',
              '/zed2/zed_node_test/pose_with_covariance', '/zed2/zed_node_test/odom',
              '/zed2/zed_node_test/left_raw/image_raw_gray/compressed', '/zed2/zed_node_test/right/camera_info',
              '/zed2/zed_node_test/left_raw/image_raw_gray', '/zed2/zed_node_test/right_raw/image_raw_gray/compressed',
              '/zed2/zed_node_test/right_raw/image_raw_gray']

    initial_timestamps = get_initial_timestamps(input_bag, topics)
    adjust_timestamp(input_bag, output_bag, initial_timestamps)
