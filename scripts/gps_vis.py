#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pyproj import Proj


class GPSPLOT:
    def __init__(self, time_slots):
        # Initialize node
        rospy.init_node("gps_plot_node")

        # Initialize subscriber to '/navsat/fix' topic
        rospy.Subscriber("/sugv/emlid_rtk_rover/tcpfix", NavSatFix, self.gps_callback)

        # Variables to store initial coordinates
        self.init_lon = None
        self.init_lat = None
        self.init_time = None

        # Create a projection for conversion
        self.proj = Proj(proj='utm', zone=33, ellps='WGS84', preserve_units=False)

        # Define time slots and corresponding colors
        self.time_slots = time_slots
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

        # Create data containers for each time slot
        self.slot_data = [{'longitude': [], 'latitude': [], 'times': []} for _ in self.time_slots]
        self.rest_data = {'longitude': [], 'latitude': [], 'times': []}

        # Create a new empty plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('GPS data')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')

        # Create a line plot for the rest of the trajectory and for each time slot
        self.rest_plot, = self.ax.plot([], [], 'o', color='gray', label='Rest of trajectory')
        self.slot_plots = [self.ax.plot([], [], 'o', color=self.colors[i % len(self.colors)],
                                        label=f'Time slot {i + 1}: {slot[0]}-{slot[1]} sec')[0]
                           for i, slot in enumerate(self.time_slots)]

        self.ax.legend()

        # Run the plot in a separate thread
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)

        plt.show()

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
                return

        # If the data point does not fall in any of the slots, append it to the rest of the trajectory
        self.rest_data['longitude'].append(lon)
        self.rest_data['latitude'].append(lat)
        self.rest_data['times'].append(t)

    def update_plot(self, i):
        # Update the data for the rest of the trajectory scatter plot
        self.rest_plot.set_data(self.rest_data['longitude'], self.rest_data['latitude'])

        # Update the data for each time slot scatter plot
        for slot_plot, slot_data in zip(self.slot_plots, self.slot_data):
            slot_plot.set_data(slot_data['longitude'], slot_data['latitude'])

        self.ax.relim()
        self.ax.autoscale_view()


if __name__ == "__main__":
    # Define time slots in seconds relative to the first message received
    time_slots = [(10, 20), (25, 35)]

    gps_plot = GPSPLOT(time_slots)

    # Keep the script running
    rospy.spin()
