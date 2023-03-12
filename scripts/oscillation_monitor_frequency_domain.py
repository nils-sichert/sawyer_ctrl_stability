#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from sensor_msgs.msg import PointCloud, JointState
from geometry_msgs.msg import Point32

class calback():
    def __init__(self) -> None:
        self.xdata_0 = 0
        self.ydata_0 = 0
        self.xdata_1 = 0
        self.ydata_1 = 0
        self.xdata_2 = 0
        self.ydata_2 = 0
        self.xdata_3 = 0
        self.ydata_3 = 0
        self.xdata_4 = 0
        self.ydata_4 = 0
        self.xdata_5 = 0
        self.ydata_5 = 0
        self.xdata_6 = 0
        self.ydata_6 = 0
        self.fig = plt.figure()
        self.ax0 = self.fig.add_subplot(2,4,1)
        self.ax0.set_xlabel('Frequency [Hz]')
        self.ax0.set_ylabel('Magnitude')
        self.ax1 = self.fig.add_subplot(2,4,2)
        self.ax1.set_xlabel('Frequency [Hz]')
        self.ax1.set_ylabel('Magnitude')
        self.ax2 = self.fig.add_subplot(2,4,3)
        self.ax2.set_xlabel('Frequency [Hz]')
        self.ax2.set_ylabel('Magnitude')
        self.ax3 = self.fig.add_subplot(2,4,4)
        self.ax3.set_xlabel('Frequency [Hz]')
        self.ax3.set_ylabel('Magnitude')
        self.ax4 = self.fig.add_subplot(2,4,5)
        self.ax4.set_xlabel('Frequency [Hz]')
        self.ax4.set_ylabel('Magnitude')
        self.ax5 = self.fig.add_subplot(2,4,6)
        self.ax5.set_xlabel('Frequency [Hz]')
        self.ax5.set_ylabel('Magnitude')
        self.ax6 = self.fig.add_subplot(2,4,7)
        self.ax6.set_xlabel('Frequency [Hz]')
        self.ax6.set_ylabel('Magnitude')

        rospy.init_node("ftt_plot", anonymous=True)
        method_ls = [self.callback_0, self.callback_1, self.callback_2, self.callback_3, self.callback_4, self.callback_5, self.callback_6]
        for i in range(7):
            topic = '/control_node_debug/oscillation_joint/' + str(i)
            rospy.Subscriber(topic, JointState, method_ls[i])

    def callback_0(self, data):
        self.xdata_0 = []*len(data.position)
        self.ydata_0 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_0 = list(data.position)
            self.ydata_0 = list(data.velocity)
    
    def callback_1(self, data):
        self.xdata_1 = []*len(data.position)
        self.ydata_1 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_1 = list(data.position)
            self.ydata_1 = list(data.velocity)
    
    def callback_2(self, data):
        self.xdata_2 = []*len(data.position)
        self.ydata_2 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_2 = list(data.position)
            self.ydata_2 = list(data.velocity)

    def callback_3(self, data):
        self.xdata_3 = []*len(data.position)
        self.ydata_3 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_3 = list(data.position)
            self.ydata_3 = list(data.velocity)

    def callback_4(self, data):
        self.xdata_4 = []*len(data.position)
        self.ydata_4 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_4 = list(data.position)
            self.ydata_4 = list(data.velocity)

    def callback_5(self, data):
        self.xdata_5 = []*len(data.position)
        self.ydata_5 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_5 = list(data.position)
            self.ydata_5 = list(data.velocity)

    def callback_6(self, data):
        self.xdata_6 = []*len(data.position)
        self.ydata_6 = []*len(data.position)
        for i in range(len(data.position)):
            self.xdata_6 = list(data.position)
            self.ydata_6 = list(data.velocity)

    def get_x(self):
        return self.xdata_0, self.xdata_1, self.xdata_2, self.xdata_3, self.xdata_4, self.xdata_5, self.xdata_6
    
    def get_y(self):
        return self.ydata_0, self.ydata_1, self.ydata_2, self.ydata_3, self.ydata_4, self.ydata_5, self.ydata_6

    def animate(self, i):
        x0, x1, x2, x3, x4, x5, x6 = plotter.get_x()
        y0, y1, y2, y3, y4, y5, y6 = plotter.get_y()
        self.ax0.clear()
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax5.clear()
        self.ax6.clear()

        self.ax0.scatter(x0, y0)
        self.ax1.scatter(x1, y1)
        self.ax2.scatter(x2, y2)
        self.ax3.scatter(x3, y3)
        self.ax4.scatter(x4, y4)
        self.ax5.scatter(x5, y5)
        self.ax6.scatter(x6, y6)


    def run(self):         
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100)
        plt.show()

plotter = calback()
plotter.run()