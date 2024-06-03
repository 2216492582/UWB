import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class DistancePlotter:
    def __init__(self):
        rospy.init_node('distance_plotter', anonymous=True)

        self.distance_data = np.zeros((8, 200))  # Initialize a buffer to store the latest 100 values for 8 distances
        self.fig, self.ax = plt.subplots()

        # Initialize the lines to be updated
        self.lines = []
        for i in range(8):
            line, = self.ax.plot(self.distance_data[i], label=f'Distance {i+1}')
            self.lines.append(line)

        self.ax.legend()
        self.ax.set_ylim(-10, 10)  # Adjust the limits as needed
        self.ax.set_xlim(0, 100)

        rospy.Subscriber("/distance_topic", Float32MultiArray, self.callback)

    def callback(self, msg):
        new_data = np.array(msg.data)
        self.distance_data = np.roll(self.distance_data, -1, axis=1)
        self.distance_data[:, -1] = new_data

    def update_plot(self):
        for i in range(8):
            self.lines[i].set_ydata(self.distance_data[i])

        self.ax.draw_artist(self.ax.patch)
        for line in self.lines:
            self.ax.draw_artist(line)

        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def run(self):
        plt.ion()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            plt.pause(0.1)
            rate.sleep()

if __name__ == '__main__':
    plotter = DistancePlotter()
    try:
        plotter.run()
    except rospy.ROSInterruptException:
        pass
