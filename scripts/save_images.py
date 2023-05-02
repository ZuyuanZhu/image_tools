#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import rosbag

class CameraSubscriber:
    def __init__(self):
        print("Hello")
        rospy.init_node('camera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_folder = '/home/zuyuan/Documents/dataset/data_cslam/sim_UGV/agent0_images'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
        self.image_count = 393

    def save_image(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            filename = os.path.join(self.image_folder, "image%04d.jpg" % self.image_count)
            cv2.imwrite(filename, cv_image)
            self.image_count += 1
            print("Image saved as {}".format(filename))
        except CvBridgeError as e:
            print(e)

    def save_images_from_rosbag(self, bagfile):
        with rosbag.Bag(bagfile, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/cam0/image_raw']):
                self.save_image(msg)

if __name__ == '__main__':
    try:
        camera_subscriber = CameraSubscriber()
        bagfile = '/home/zuyuan/Documents/dataset/data_cslam/agent1_sim.bag'
        camera_subscriber.save_images_from_rosbag(bagfile)
    except rospy.ROSInterruptException:
        pass

