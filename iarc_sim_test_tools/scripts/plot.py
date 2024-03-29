#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped

def plot_x(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(time ,msg.pose.position.x, '*')
	plt.xlabel("time")
	plt.ylabel("value")
	plt.ylim(-0.1 , 0.1)
        #plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("/rpy_values", PoseStamped, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()
