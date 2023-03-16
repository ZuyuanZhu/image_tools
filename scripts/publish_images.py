#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import os
import pykitti
import rosbag


class CameraPublisher:

    def __init__(self, base_path_, sequence_, verbose_):
        rospy.init_node('camera_publisher', anonymous=True)
        self.verbose = verbose_
        self.pub_cam0 = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)
        self.pub_cam1 = rospy.Publisher('/cam1/image_raw', Image, queue_size=10)
        self.pub_pose = rospy.Publisher('/leica/position', PoseStamped, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.base_path = base_path_
        self.sequence = sequence_

        self.pose_path = os.path.join(self.base_path, 'poses', self.sequence + '.txt')
        self.image_folder = os.path.join(self.base_path, 'sequences', self.sequence)
        self.img_folder_0 = os.path.join(self.image_folder, 'image_0')
        self.img_folder_1 = os.path.join(self.image_folder, 'image_1')
        self.image_files_0 = sorted(os.listdir(self.img_folder_0))
        self.image_files_1 = sorted(os.listdir(self.img_folder_1))
        self.num_images_0 = len(self.image_files_0)
        self.num_images_1 = len(self.image_files_1)
        self.current_image_index_0 = 0
        self.current_image_index_1 = 0
        self.current_pos_index = 0

        self.bag_path = os.path.join(self.base_path, 'bags', self.sequence + '.bag')
        self.bag = rosbag.Bag(self.bag_path, 'w')

    def publish_data(self, pos_xyz_):
        while not rospy.is_shutdown() and (self.current_image_index_0 != self.num_images_0):
            image_filename_0 = os.path.join(self.img_folder_0, self.image_files_0[self.current_image_index_0])
            image_filename_1 = os.path.join(self.img_folder_1, self.image_files_1[self.current_image_index_1])

            # Create a pose data message
            pose_msg = PoseStamped()
            # Set the pose data values
            pose_msg.pose.position.x = pos_xyz_[self.current_image_index_0][0]
            pose_msg.pose.position.y = pos_xyz_[self.current_image_index_0][1]
            pose_msg.pose.position.z = pos_xyz_[self.current_image_index_0][2]
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            # Set the pose data header values
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"

            try:
                img_0 = cv2.imread(image_filename_0, cv2.IMREAD_COLOR)
                img_1 = cv2.imread(image_filename_1, cv2.IMREAD_COLOR)
                ros_image_0 = self.bridge.cv2_to_imgmsg(img_0, "bgr8")
                ros_image_1 = self.bridge.cv2_to_imgmsg(img_1, "bgr8")
                self.pub_cam0.publish(ros_image_0)
                self.pub_cam1.publish(ros_image_1)
                self.pub_pose.publish(pose_msg)
                self.loginfo("Current image index is %d/%d " %
                             (self.current_image_index_0, self.num_images_0))

                # Write the ROS image message to the bag file
                self.bag.write('/cam0/image_raw', ros_image_0)
                self.bag.write('/cam1/image_raw', ros_image_1)
                self.bag.write('/leica/position', pose_msg)

                self.rate.sleep()
            except CvBridgeError as e:
                print(e)

            self.current_image_index_0 = self.current_image_index_0 + 1
            self.current_image_index_1 = self.current_image_index_1 + 1

    def cleanup(self):
        self.bag.close()

    def loginfo(self, msg):
        if self.verbose:
            print(msg)


if __name__ == '__main__':

    base_path = '/home/zuyuan/rasberry_ws/src/data_kitti/dataset/'
    sequence = '04'
    verbose = True
    camera_publisher = CameraPublisher(base_path, sequence, verbose)

    try:
        # Load the data. Optionally, specify the frame range to load.
        # Each row of the file contains the first 3 rows of a 4x4 homogeneous
        # pose matrix flattened into one line.
        # It means that this matrix:
        # r11 r12 r13 tx
        # r21 r22 r23 ty
        # r31 r32 r33 tz
        # 0   0   0   1
        dataset = pykitti.odometry(base_path, sequence)

        pos_xyz = []
        for p in dataset.poses:
            # only take tx, ty, tz
            p_xyz = p[0:3, -1]
            pos_xyz.append(p_xyz)

        camera_publisher.publish_data(pos_xyz)

    except rospy.ROSInterruptException:
        pass

    finally:
        camera_publisher.cleanup()
