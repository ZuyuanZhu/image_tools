import rosbag


def rename(input_bag, output_bag0, output_bag1):
    # open the bag file for reading
    with rosbag.Bag(input_bag, 'r') as bag:
        # create a new bag file for writing
        with rosbag.Bag(output_bag0, 'w') as outbag0:
            # iterate through the messages in the input bag
            for topic, msg, t in bag.read_messages():
                # change the name of the topic
                if topic == '/LUGV/groundtruth/values':
                    topic = '/leica/position'
                elif topic == '/LUGV/inertial_unit/quaternion':
                    topic = '/imu0'
                elif topic == '/LUGV/stereo_camera_left_camera/image':
                    topic = '/cam0/image_raw'
                else:
                    continue
                # write the message to the output bag
                outbag0.write(topic, msg, t)

        with rosbag.Bag(output_bag1, 'w') as outbag1:
            for topic, msg, t in bag.read_messages():
                if topic == '/SUGV/groundtruth/values':
                    topic = '/leica/position'
                elif topic == '/SUGV/inertial_unit/quaternion':
                    topic = '/imu0'
                elif topic == '/SUGV/stereo_camera_left_camera/image':
                    topic = '/cam0/image_raw'
                else:
                    continue
                outbag1.write(topic, msg, t)


if __name__ == "__main__":

    in_bag = '/home/zuyuan/rasberry_ws/src/cslam_datasets/sim/aided_sim.bag'
    output_bag0 = '/home/zuyuan/rasberry_ws/src/cslam_datasets/sim/agent0_sim.bag'
    output_bag1 = '/home/zuyuan/rasberry_ws/src/cslam_datasets/sim/agent1_sim.bag'
    rename(in_bag, output_bag0, output_bag1)
