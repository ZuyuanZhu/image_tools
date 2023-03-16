#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraSubscriber:
    def __init__(self):
        print("Hello")
        rospy.init_node('camera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/cam0/image_raw', Image, self.image_callback)
        self.image_folder = '/home/zuyuan/rasberry_ws/src/cslam_datasets/MH_01_easy_images'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
        self.image_count = 0

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            filename = os.path.join(self.image_folder, "image%04d.jpg" % self.image_count)
            cv2.imwrite(filename, cv_image)
            self.image_count += 1
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        camera_subscriber = CameraSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
