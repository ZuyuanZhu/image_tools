#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import rosbag


class CameraPublisher:

    def __init__(self, base_path_, verbose_):
        rospy.init_node('camera_publisher', anonymous=True)
        self.verbose = verbose_
        self.pub_cam0 = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.base_path = base_path_

        self.img_folder = self.base_path
        self.image_files = sorted(os.listdir(self.img_folder))
        self.num_images_0 = len(self.image_files)
        self.current_image_index = 0
        self.current_pos_index = 0

        self.bag_path = os.path.join(self.base_path, 'bags' + '.bag')
        self.bag = rosbag.Bag(self.bag_path, 'w')

    def publish_data(self):
        while not rospy.is_shutdown() and (self.current_image_index != self.num_images_0):
            image_filename = os.path.join(self.img_folder, self.image_files[self.current_image_index])

            try:
                img_0 = cv2.imread(image_filename, cv2.IMREAD_COLOR)
                ros_image = self.bridge.cv2_to_imgmsg(img_0, "bgr8")
                self.pub_cam0.publish(ros_image)
                self.loginfo("Current image index is %d/%d " %
                             (self.current_image_index, self.num_images_0))

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

    base_path = '/media/zuyuan/DATA1TB/kitti/2011_10_03/2011_10_03_drive_0034_sync/image_00/test'
    verbose = True
    camera_publisher = CameraPublisher(base_path, verbose)

    try:
        camera_publisher.publish_data()

    except rospy.ROSInterruptException:
        pass

    finally:
        camera_publisher.cleanup()
