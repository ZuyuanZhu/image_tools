#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pyproj import Proj


class GPSPLOT:
    def __init__(self):
        # Initialize node
        rospy.init_node("gps_plot_node")

        # Initialize subscriber to '/navsat/fix' topic
        rospy.Subscriber("/navsat/fix", NavSatFix, self.gps_callback)

        # Initialize the variables that will hold the longitude and latitude
        self.longitude = []
        self.latitude = []

        # Create a projection for conversion
        self.proj = Proj(proj='utm', zone=33, ellps='WGS84', preserve_units=False)

        # Variables to store initial coordinates
        self.init_lon = None
        self.init_lat = None

        # Create a new empty plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('GPS data')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.scatter, = self.ax.plot([], [], 'o')

        # Run the plot in a separate thread
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)

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

    def update_plot(self, i):
        self.scatter.set_data(self.longitude, self.latitude)
        self.ax.relim()
        self.ax.autoscale_view()
        return self.scatter,


if __name__ == "__main__":
    gps_plot = GPSPLOT()

    # Keep the script running
    rospy.spin()
