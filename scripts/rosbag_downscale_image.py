#!/usr/bin/env python

import rospy
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def resize_image(image_msg, bridge):
    # Convert the image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

    # Resize the image
    resized_image = cv2.resize(cv_image, (640, 360))

    # Convert the OpenCV image back to an image message
    resized_image_msg = bridge.cv2_to_imgmsg(resized_image, encoding='passthrough')

    # Keep the header from the original image
    resized_image_msg.header = image_msg.header

    return resized_image_msg


if __name__ == "__main__":
    rospy.init_node("downscale_rosbag")

    bridge = CvBridge()
    base_path = "/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/"
    in_bag_name = "test_2_10_55_select_frame_id_odom"
    in_bag = rosbag.Bag(base_path + in_bag_name + ".bag")
    out_bag_name = in_bag_name + "_downscale"
    out_bag = rosbag.Bag(base_path + out_bag_name + ".bag", 'w')

    try:
        for topic, msg, t in in_bag.read_messages():
            if topic == "/zed2/zed_node_test/left_raw/image_raw_gray":
                try:
                    # Resize the image
                    resized_image_msg = resize_image(msg, bridge)

                    # Write the resized image message to the new bag
                    out_bag.write(topic, resized_image_msg, t)
                except CvBridgeError as e:
                    print(e)
            else:
                # For non-image data, we just add it to the new bag without modification
                out_bag.write(topic, msg, t)
    finally:
        in_bag.close()
        out_bag.close()
