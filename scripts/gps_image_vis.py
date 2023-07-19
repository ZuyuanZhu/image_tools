#!/usr/bin/env python3
# run from terminal: python3 gps_image_vis.py
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
from pyproj import Proj
import matplotlib
matplotlib.use('TkAgg')


class GPSPLOT:
    def __init__(self):
        # Initialize node
        rospy.init_node("gps_plot_node")

        # Initialize image converter
        self.bridge = CvBridge()

        # Initialize subscriber to '/navsat/fix' topic
        rospy.Subscriber("/sugv/emlid_rtk_rover/tcpfix", NavSatFix, self.gps_callback)

        # Initialize subscriber to '/zed2/zed_node_test/right_raw/image_raw_gray' topic
        rospy.Subscriber("/zed2/zed_node_test/right_raw/image_raw_gray", Image, self.image_callback, queue_size=10)

        # Initialize the variables that will hold the longitude and latitude
        self.longitude = []
        self.latitude = []

        # Create a projection for conversion
        self.proj = Proj(proj='utm', zone=33, ellps='WGS84', preserve_units=False)

        # Variables to store initial coordinates
        self.init_lon = None
        self.init_lat = None

        # Create a new empty plot
        self.fig, self.axs = plt.subplots(1, 2)
        self.axs[0].set_title('GPS data')
        self.axs[0].set_xlabel('East (m)')
        self.axs[0].set_ylabel('North (m)')
        self.scatter, = self.axs[0].plot([], [], 'o')

        # Create placeholder image
        self.current_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image_plot = self.axs[1].imshow(self.current_image)

        # Run the plot in a separate thread
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=25)

        plt.show()

    def gps_callback(self, msg):
        # Extract longitude and latitude from the GPS fix message and convert to UTM
        x, y = self.proj(msg.longitude, msg.latitude)

        # If initial coordinates are not stored, store them
        if self.init_lon is None and self.init_lat is None:
            self.init_lon = x
            self.init_lat = y

        # Subtract initial coordinates to get local coordinates and append to lists
        self.longitude.append(x - self.init_lon)
        self.latitude.append(y - self.init_lat)

        self.gps_data_received = True

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.current_image = cv_image

        self.image_data_received = True

    def update_plot(self, i):
        if hasattr(self, 'gps_data_received'):
            self.scatter.set_data(self.longitude, self.latitude)
            self.axs[0].relim()
            self.axs[0].autoscale_view()
            self.axs[0].set_aspect('equal', adjustable='datalim')


        if hasattr(self, 'image_data_received'):
            self.image_plot.set_data(cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB))
            self.axs[1].imshow(self.current_image, cmap='gray')
            self.axs[1].axis('off')

        self.fig.canvas.draw()  # Force update of the plot

        return self.scatter, self.image_plot


if __name__ == "__main__":
    gps_plot = GPSPLOT()

    # Keep the script running
    rospy.spin()
