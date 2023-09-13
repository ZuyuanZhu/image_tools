#!/usr/bin/env python
from rosbag.bag import Bag
import rospy


def split_bag(input_bag, output_bag, start_time_sec, end_time_sec, sync_topics):
    print(f"Reading rosbag: {input_bag}")

    latched_topics = {}
    time_ref = None  # For storing the reference timestamp from tf_static

    # Calculate absolute start and end times
    with Bag(input_bag, 'r') as bag:
        start_time = bag.get_start_time() + start_time_sec
        end_time = bag.get_start_time() + end_time_sec
        for topic, msg, t in Bag(input_bag, 'r').read_messages():
            if topic == '/tf_static':
                # If time_ref is not yet set, grab the first timestamp as reference
                if time_ref is None:
                    time_ref = msg.transforms[0].header.stamp
                    break

    # Open the output bag
    num_tf = 0
    start_times = {}
    with Bag(output_bag, 'w') as outbag:
        # Read the input bag
        for topic, msg, t in Bag(input_bag, 'r').read_messages():
            if topic == '/tf_static' and num_tf < 2:
                # If time_ref is not yet set, grab the first timestamp as reference
                for i in range(len(msg.transforms)):
                    if abs(msg.transforms[i].header.stamp.to_sec() - time_ref.to_sec()) > 0.01:
                        # delta = trans.header.stamp.to_sec() - time_ref.to_sec()
                        msg.transforms[i].header.stamp.secs = time_ref.secs
                        a = msg.transforms[i].header.stamp.nsecs
                        b = time_ref.nsecs
                        b_top2 = int(b/10000000)
                        a_rest2 = a % 10000000
                        msg.transforms[i].header.stamp.nsecs = b_top2 * 10000000 + a_rest2

                # only keep the first two msg
                corret_time = t.to_sec() + start_time_sec
                t = rospy.Time.from_sec(corret_time)
                outbag.write(topic, msg, t)
                num_tf += 1
                continue
            else:
                if start_time <= t.to_sec() <= end_time:
                    if msg._has_header:
                        if topic in sync_topics:
                            if topic not in start_times.keys():
                                start_times[topic] = msg.header.stamp.to_sec()
                            stamp_temp = msg.header.stamp.to_sec() - start_times[topic] + time_ref.to_sec()
                            msg.header.stamp = rospy.Time.from_sec(stamp_temp)
                            # corret_time = t.to_sec() - start_time_sec
                            # t = rospy.Time.from_sec(corret_time)
                            outbag.write(topic, msg, t)
                    elif topic == '/tf_static' or '/tf':
                        # If time_ref is not yet set, grab the first timestamp as reference
                        for i in range(len(msg.transforms)):
                            if topic not in start_times.keys():
                                start_times[topic] = msg.transforms[i].header.stamp.to_sec()
                            stamp_temp = msg.transforms[i].header.stamp.to_sec() - start_times[topic] + time_ref.to_sec()
                            msg.transforms[i].header.stamp = rospy.Time.from_sec(stamp_temp)
                        # corret_time = t.to_sec() - start_time_sec
                        # t = rospy.Time.from_sec(corret_time)
                        outbag.write(topic, msg, t)
                    else:
                        # For other topics, adjust the timestamp based on the reference
                        raise ValueError("Undefined topics in rosbag!")
                elif t.to_sec() > end_time:
                    break
    print(f"New bag saved: {output_bag}")


if __name__ == '__main__':
    base_path = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/'
    src_bag = base_path + 'test_2_2023-07-04-10-13-18_select_tf.bag'
    start_time = 10
    end_time = 55
    output_bag = base_path + 'test_2_tf_%d_%d.bag' % (start_time, end_time)
    sync_topics = ['/sugv/velodyne_points',
                   '/zed2/zed_node_test/odom',
                   '/zed2/zed_node_test/left_raw/image_raw_gray']
    split_bag(src_bag, output_bag, start_time, end_time, sync_topics)
