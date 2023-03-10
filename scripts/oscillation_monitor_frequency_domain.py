import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class calback():
    def __init__(self) -> None:
        self.xdata = 0
        self.ydata = 0
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)
        self.ax1.set_ylabel("Power")
        self.ax1.set_xlabel("Frequency [Hz]")
        rospy.init_node("ftt_plot", anonymous=True)
        rospy.Subscriber("/control_node_debug/oscillation_joint1", PointCloud, self.callback)

    def callback(self, data):
        self.xdata = []
        self.ydata = []
        for i in range(len(data.points)):
            self.xdata.append(data.points[i].x)
            self.ydata.append(data.points[i].y) 

    def get_x(self):
        return self.xdata
    
    def get_y(self):
        return self.ydata

    def animate(self, i):
        xs = plotter.get_x()
        ys = plotter.get_y()
        self.ax1.clear()
        self.ax1.bar(xs, ys)

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=10000)
        plt.show()

plotter = calback()
plotter.run()