#!/usr/bin/env python
import rospy

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Vector3


class Visualiser:
    def __init__(self):
        # self.fig, self.ax = plt.subplots()
        # self.ln = plt.plot([], [], 'ro')
        # self.x_data, self.y_data = [] , []
        self._x = []
        self._y = []
        self._z = []

        self.x_data, self.y_data, self.z_data = [], [], []
        self.fig = plt.figure()  
        self.ax = self.fig.add_subplot(111, projection='3d')  
        self.ax.plot(self.x_data, self.y_data, self.z_data,
                                      label='Trayectoria simulada')
        self.ax.plot([0,4,4,0,0], [0,0,4,4,0], [1,1,1,1,1], 
                                      label='Trayectoria deseada')

    def plot_init(self):
        # self.ax.set_xlim(0, 10000)
        # self.ax.set_ylim(-7, 7)
        # return self.ln
        self.ax.set(xlim=(-1, 5), ylim=(-1, 5), zlim=(0, 1.5))
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_zlabel('z [m]')
        self.ax.legend()

    def odom_callback(self, msg):
        self._x = msg.x
        self._y = msg.y
        self._z = msg.z
        print("x: {} y: {} z: {}".format(self._x, self._y, self._z))
        # self.x_data.append(msg.x)
        # self.y_data.append(msg.y)
        # x_index = len(self.x_data)
        # self.x_data.append(x_index+1)
    
    def update_plot(self, frame):
        # self.ln.set_data(self.x_data, self.y_data)
        # return self.ln
        self.x_data.append(self._x)
        self.y_data.append(self._y)
        self.z_data.append(self._z)
        self.ax.plot(self.x_data, self.y_data, self.z_data)
        
        # return self.ax


if __name__=='__main__':

    try:
        while not rospy.is_shutdown():
            rospy.init_node('plot_3d_node')
            vis = Visualiser()
            sub = rospy.Subscriber('/bebopose', Vector3, vis.odom_callback)

            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
            plt.show(block=True) 
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))