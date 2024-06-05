#!/usr/bin/env python

import rospy
from nlink_distance.msg import DistanceArray_oneself  # Import the custom message type
import matplotlib.pyplot as plt
import numpy as np

class DistancePlotter:
    def __init__(self):
        self.buffer_size = 100
        self.distance_data = np.zeros((8, self.buffer_size))  # Initialize a buffer to store the latest 100 values for 8 distances
        self.index = 0
        self.fig, self.ax = plt.subplots()

        # Initialize the lines to be updated
        self.lines = []
        for i in range(8):
            line, = self.ax.plot(self.distance_data[i], label=f'Distance {i+1}')
            self.lines.append(line)

        self.ax.legend()
        self.ax.set_ylim(0.5, 1.2)  # Adjust the limits as needed
        self.ax.set_xlim(0, self.buffer_size)

    def callback(self, msg):
        new_data = np.array(msg.distances)
        if new_data.size == 8:
            self.distance_data[:, self.index] = new_data
            self.index = (self.index + 1) % self.buffer_size

    def update_plot(self):
        for i in range(8):
            # print(1)
            if self.index == 0:
                # When the index is at the start, use the whole buffer
                self.lines[i].set_ydata(self.distance_data[i])
            else:
                # When the index is not at the start, split and concatenate for continuous plot
                self.lines[i].set_ydata(np.concatenate((self.distance_data[i, self.index:], self.distance_data[i, :self.index])))

        self.fig.canvas.draw()  # Corrected method to refresh the canvas
        self.fig.canvas.flush_events()

    def run(self):
        plt.ion()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            plt.pause(0.1)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('distance_plotter', anonymous=True)
    plotter = DistancePlotter()
    rospy.Subscriber("/distance_topic", DistanceArray_oneself, plotter.callback)
    try:
        plotter.run()
    except rospy.ROSInterruptException:
        pass
