#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pyproj import Proj
from nav_msgs.msg import Odometry


class GPSPLOT:
    def __init__(self, time_slots, specific_timestamps):
        # Initialize node
        rospy.init_node("gps_plot_node")

        # Initialize subscriber to '/navsat/fix' topic
        rospy.Subscriber("/sugv/emlid_rtk_rover/tcpfix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/sugv/odometry/filtered", Odometry, self.odom_callback)

        self.odom_data = {'longitude': [], 'latitude': []}

        # Variables to store initial coordinates
        self.init_lon = None
        self.init_lat = None
        self.init_time = None
        self.init_odom_x = None
        self.init_odom_y = None

        # Create a projection for conversion
        self.proj = Proj(proj='utm', zone=33, ellps='WGS84', preserve_units=False)

        # Define time slots and corresponding colors
        self.time_slots = time_slots
        self.colors = ['b', 'y', 'r', 'c', 'm', 'g', 'k']

        # Create data containers for each time slot
        self.slot_data = [{'longitude': [], 'latitude': [], 'times': []} for _ in self.time_slots]
        self.rest_data = {'longitude': [], 'latitude': [], 'times': []}

        # Create a new empty plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('SUGV Odometry VS RTK GPS')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')

        # Create a line plot for the rest of the trajectory and for each time slot
        self.rest_plot, = self.ax.plot([], [], 'o', color='gray', markersize=4, label='GPS')
        self.odom_plot, = self.ax.plot([], [], 'o-', color='purple', markersize=4, label='Odometry')

        self.slot_plots = [self.ax.plot([], [], 'o', color=self.colors[i % len(self.colors)],
                                        label=f'Time slot {i + 1}: {slot[0]}-{slot[1]} sec')[0]
                           for i, slot in enumerate(self.time_slots)]

        # Add specific timestamps and corresponding plot objects
        self.specific_timestamps = specific_timestamps
        self.special_points = [[] for _ in self.specific_timestamps]
        self.point_plots = [self.ax.plot([], [], marker['style'], color=marker['color'], markersize=10,
                                         label=f'LCD Matched Frame {i + 1}')[0]
                            for i, marker in enumerate(self.specific_timestamps)]

        self.ax.legend()

        # Run the plot in a separate thread
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)

        plt.show()

    def odom_callback(self, msg):
        # Extract x, y from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # If initial odometry readings are not stored, store them
        if self.init_odom_x is None and self.init_odom_y is None:
            self.init_odom_x = x
            self.init_odom_y = y

        # Subtract initial readings to get local coordinates
        x -= self.init_odom_x
        y -= self.init_odom_y

        self.odom_data['longitude'].append(x)
        self.odom_data['latitude'].append(y)

    def gps_callback(self, msg):
        # Extract longitude, latitude and time from the GPS fix message, and convert to UTM
        x, y = self.proj(msg.longitude, msg.latitude)
        t = msg.header.stamp.to_sec()

        # If initial coordinates are not stored, store them
        if self.init_lon is None and self.init_lat is None and self.init_time is None:
            self.init_lon = x
            self.init_lat = y
            self.init_time = t

        # Subtract initial coordinates to get local coordinates
        lon = x - self.init_lon
        lat = y - self.init_lat
        t = t - self.init_time

        # Check in which time slot (if any) the current data point falls
        for slot_data, slot in zip(self.slot_data, self.time_slots):
            if slot[0] <= t < slot[1]:
                slot_data['longitude'].append(lon)
                slot_data['latitude'].append(lat)
                slot_data['times'].append(t)

        # If the data point does not fall in any of the slots, append it to the rest of the trajectory
        self.rest_data['longitude'].append(lon)
        self.rest_data['latitude'].append(lat)
        self.rest_data['times'].append(t)

        # Check if the timestamp matches any of the special timestamps
        for i, marker in enumerate(self.specific_timestamps):
            # print("marker['time']: {}, t: {}".format(marker['time'], t))
            if marker['time'] - 0.1 <= t < marker['time'] + 0.1:  # add some tolerance
                # print("lon: {}, lat: {}, t: {}".format(lon, lat, i))
                self.special_points[i].append((lon, lat))

    def update_plot(self, i):
        # Update the data for the rest of the trajectory scatter plot
        self.rest_plot.set_data(self.rest_data['longitude'], self.rest_data['latitude'])
        self.odom_plot.set_data(self.odom_data['longitude'], self.odom_data['latitude'])

        # Update the data for each time slot scatter plot
        for slot_plot, slot_data in zip(self.slot_plots, self.slot_data):
            slot_plot.set_data(slot_data['longitude'], slot_data['latitude'])

        # Update the data for each specific point
        for point_plot, points in zip(self.point_plots, self.special_points):
            if points:  # Check if there are any points for this timestamp
                lon, lat = zip(*points)  # Unzip the list of points into separate lists of longitudes and latitudes
                point_plot.set_data(lon, lat)

        self.ax.relim()
        self.ax.set_aspect('equal')
        self.ax.autoscale_view()
        self.ax.set_aspect('equal', 'box')


if __name__ == "__main__":
    time_slots = []  # rosbag 2, used for highlight sub-trajectory, e.g, (0, 127)

    specific_timestamps = []

    gps_plot = GPSPLOT(time_slots, specific_timestamps)

    # Keep the script running
    rospy.spin()
