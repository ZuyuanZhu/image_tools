#!/usr/bin/env python

from cv2 import IMREAD_COLOR, imread
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import rosbag
from evo.tools import file_interface
import re
from pathlib import Path
from copy_images_between_range import copy_images_between_range



class CameraPublisher:

    """
    This script is a Python script for a ROS node that publishes images from a specified
    folder as a camera topic. The images are read from a folder,and their corresponding
    timestamps are read from a separate text file. The script also writes the published
    image messages to a ROS bag file for later use.
    """
    def __init__(self, base_path_, folder_, time_stamps_, verbose_):
        rospy.init_node('camera_publisher', anonymous=True)
        self.verbose = verbose_
        self.pub_cam0 = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.base_path = base_path_

        self.img_folder = os.path.join(self.base_path, folder_)
        self.image_files = sorted(os.listdir(self.img_folder))
        self.num_images = len(self.image_files)
        self.current_image_index = 0
        self.current_pos_index = 0

        # Create the destination folder if it doesn't exist
        self.bag_folder_path = os.path.join(self.base_path, 'output_frame_id', folder_)
        if not Path(self.bag_folder_path).exists():
            Path(self.bag_folder_path).mkdir()

        self.bag_path = os.path.join(self.bag_folder_path, 'bags' + '.bag')
        self.bag = rosbag.Bag(self.bag_path, 'w')

        self.time_stamps = time_stamps_

    def time_start_idx(self):
        """
        Get the time stamp index of the first image
        """
        m = re.search(r'\d+', self.image_files[0])
        if m:
            number = m.group()
            return number
        else:
            print('No number found')
            exit(1)

    def publish_data(self):

        raw_timestamps_mat = file_interface.csv_read_matrix(self.time_stamps)
        seq = 0
        while not rospy.is_shutdown() and (self.current_image_index != self.num_images):
            image_filename = os.path.join(self.img_folder, self.image_files[self.current_image_index])

            try:
                img_0 = imread(image_filename, IMREAD_COLOR)
                ros_image = self.bridge.cv2_to_imgmsg(img_0, "bgr8")

                time_start_idx = int(self.time_start_idx())
                # print("Time stamp idx: %d" % time_start_idx)
                # Set the time stamp for the message header
                # timestamp = rospy.Time.now()   # not good for tracking, should use the timestamp provided by the dataset
                time_float = float(raw_timestamps_mat[time_start_idx + seq][0])
                # Convert the float to a rospy.Time object
                secs = int(time_float)
                nsecs = int((time_float - secs) * 1e9)
                ros_time = rospy.Time(secs, nsecs)
                ros_image.header.stamp = ros_time
                # print("raw_timestamps_mat[time_start_idx+seq]: %f" % float(raw_timestamps_mat[time_start_idx + seq][0]))
                # Set the sequence number for the message header
                seq += 1
                ros_image.header.seq = seq

                # Set the frame_id of the header to the name of the image file
                ros_image.header.frame_id, _ = os.path.splitext(self.image_files[self.current_image_index])

                self.pub_cam0.publish(ros_image)
                self.loginfo("Current image index is %d/%d " %
                             (self.current_image_index, self.num_images))

                # Write the ROS image message to the bag file
                self.bag.write('/cam0/image_raw', ros_image)
                self.rate.sleep()
            except CvBridgeError as e:
                print(e)

            self.current_image_index = self.current_image_index + 1

    def cleanup(self):
        self.bag.close()

    def loginfo(self, msg):
        if self.verbose:
            print(msg)


if __name__ == '__main__':

    base_path = '/media/zuyuan/DATA1TB/kitti/kitti_splited/2011_10_03_0034_seq02/'
    folder = 'line2-3.5'
    source_folder = '/media/zuyuan/DATA1TB/kitti/2011_10_03/2011_10_03_drive_0034_sync/image_00/data/'
    start = 688
    end = 1165
    dest_folder = base_path + folder

    copy_images_between_range(source_folder, start, end, dest_folder)
    print("Images have been copied to the %s folder." % dest_folder)

    time_stamps = '/home/zuyuan/Documents/dataset/kitti/dataset/sequences/02/times.txt'
    verbose = True
    camera_publisher = CameraPublisher(base_path, folder, time_stamps, verbose)

    try:
        camera_publisher.publish_data()

    except rospy.ROSInterruptException:
        pass

    finally:
        camera_publisher.cleanup()
