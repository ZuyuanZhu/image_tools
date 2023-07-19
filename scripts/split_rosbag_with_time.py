#!/usr/bin/env python
from rosbag.bag import Bag


def split_bag(input_bag, output_bag, start_time_sec, end_time_sec):
    start_time = None
    end_time = None

    print(f"Reading rosbag: {input_bag}")

    # Get start time of the bag
    with Bag(input_bag, 'r') as bag:
        start_time = bag.get_start_time() + start_time_sec
        end_time = bag.get_start_time() + end_time_sec

    # Split the bag
    with Bag(output_bag, 'w') as outbag:
        for topic, msg, t in Bag(input_bag, 'r').read_messages():
            if hasattr(msg, 'header'):
                seq = msg.header.seq
            else:
                seq = None

            if start_time <= t.to_sec() <= end_time:
                print(f"Write msg seq: {seq}")
                outbag.write(topic, msg, t)
            elif start_time > t.to_sec():
                print(f"Read msg seq: {seq}")
            else:
                break

    print(f"New bag saved: {output_bag}")


if __name__ == '__main__':
    base_path = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/'
    src_bag = base_path + 'test_4_2023-07-04-10-32-34.bag'
    start_time = 73
    end_time = 116
    output_bag = base_path + 'test_4_%d_%d.bag' % (start_time, end_time)
    split_bag(src_bag, output_bag, start_time, end_time)
