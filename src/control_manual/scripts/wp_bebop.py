#!/usr/bin/env python
from time import time, strftime

import rospy
import tf
import math
import json
import matplotlib.pyplot as plt
from uuid import uuid4
from math import atan2, sqrt, cos, sin, pi
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from nav_msgs.msg import Odometry 
import numpy as np

import waypoints as wp

UNIQUE_ID = uuid4()
HOVER = False

# WAYPOINTS = wp.WAYPOINTS
WAYPOINTS = [
    (0,   0, 1),
    (1.5, 0, 1),
    (1.5, 1.5, 1),
    (0,  1.5, 1),
    (0,  0 ,1)
]

# WAYPOINTS = [
#     (0.0, 0.0, 1, 0.0),
#     (0.0, 0.0, 1, 1.5708),
#     (1.5, 0.0, 1, 1.5708),
#     (1.5, 0.0, 1, 3.1416),
#     (1.5, 1.5, 1, 3.1416),
#     (1.5, 1.5, 1, 4.7124),
#     (0.0, 1.5, 1, 4.7124),
#     (0.0, 1.5, 1, 6.2832),
#     (0.0, 0.0 ,1, 6.2832)
]

# x_plot = {
#     'error': [],
#     'time': []
# }
# y_plot = {
#     'error': [],
#     'time': []
# }
# z_plot = {
#     'error': [],
#     'time': []
# }
x_plot = []
y_plot = []
z_plot = []
time = []
   

# function to add to JSON 
def write_json(new_data, filename='data.json'): 
    with open(filename) as json_file: 
        data = json.load(json_file) 
        
        temp = data['PID'] 
   
        # appending data to emp_details  
        temp.append(new_data)

    with open(filename,'w') as f: 
        json.dump(data, f, indent=4)



class PID(object):
    """PID class returns the pid error
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prevTime = None
        self.prevError = 0
        self.cumError = 0

    def calculate_pid(self, error):
        cur_time = rospy.get_time()
        if self.prevTime == None:
            self.prevTime = cur_time
            self.prevError = error
            return self.kp * error
        dt = cur_time - self.prevTime
        self.prevTime = cur_time
        self.cumError += error * dt
        pid = self.kp * error + self.kd * 
            (error - self.prevError) / dt + \
            self.ki * self.cumError
        self.prevError = error
        return pid


class Bebop_functions():
    def __init__(self):
        # Bebop_control node creation
        rospy.init_node('Bebop_control', anonymous=True)

        # Publisher which will publish to the topic '/bebop/takeoff'.
        self.takeoff_publisher = rospy.Publisher('/bebop/takeoff',
                                                 Empty, queue_size=10)

        # Publisher which will publish to the topic '/bebop/land'.
        self.landing_publisher = rospy.Publisher('/bebop/land',
                                                 Empty, queue_size=10)

        # Publisher which will publish to the topic '/bebop/cmd_vel'.
        self.cmdvel_publisher = rospy.Publisher('/bebop/cmd_vel',
                                                Twist, queue_size=10)

        # A subscriber to the topic '/bebop/states/ardrone3/PilotingState/FlyingStateChanged'. self.update_state is called
        # when a message of type Ardrone3PilotingStateFlyingStateChanged is received.
        self.pilotingstate_subscriber = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
                                                         Ardrone3PilotingStateFlyingStateChanged, self.update_state)

        # Utiliza un sistema de sensores externos para calcular la pose del drone
        # Subscriber to the vicon transform node to read the position and quaternions of the bebop2
        # self.bebopcoord_subscriber = rospy.Subscriber(
        #     'vicon/Bepop2020/Bepop2020', TransformStamped, self.bebop_pose)

        # Nuevo se toma la pose a traves de la odometria del drone Bebop2
        self.bebopcoord_subscriber = rospy.Subscriber("/bebop/odom", Odometry, self.bebop_pose)

        self.state = None
        self.yaw = 0
        self.bebopose = Vector3()

        self.controlX = PID(0.8, 0.001, 1.5)  # PID(0.8, 0.001, 1.5)
        self.controlY = PID(0.8, 0.001, 1.5)
        self.controlZ = PID(0.12, 0.01, 0.25)  #PID(0.12, 0.01, 0.25)

        self.controlYAW = PID(0.12, 0.01, 0.25)  #PID(0.12, 0.01, 0.25)

        self.vel_lim = 0.3  # 2.0
        self.finished = False
        self.waypoint = WAYPOINTS.pop(0)
        # TransformStamped message update rate limited to 5 Hz, which limits to 5 Hz the subscribing rate.
        self.rate = rospy.Rate(10)

        # Escritura de los valores producidos (odometria, PID)
        filename = 'datos_' + strftime('%d_%b_%Y_%H_%M_%S') + '.csv'
        self.logfile = open(filename, 'w')
        self.write_and_flush('PoseX, PoseY, PoseZ,')
        self.write_and_flush('errorX, errorY, errorZ, tiempo\n')

    def takeoff(self):  # Take off command function
        while self.state != 2 and not rospy.is_shutdown():
            hello_str = "Takeoff command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.takeoff_publisher.publish(Empty())
            self.rate.sleep()
        print("Takeoff is done!")

    def land(self):  # Land command function
        self.finished = True
        self.cmdvel_publisher.publish(Twist())
        while self.state != 0 and not rospy.is_shutdown():
            hello_st = "Land command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_st)
            self.landing_publisher.publish(Empty())
            self.rate.sleep()

        print("Landing completed succesfully!")

    def move(self):  # Move function with a P controller.

        twist = Twist()

        while not rospy.is_shutdown() and not self.finished:

            delta_x = self.waypoint[0] - self.bebopose.position.x  # position error
            delta_y = self.waypoint[1] - self.bebopose.position.y
            delta_z = self.waypoint[2] - self.bebopose.position.z
            delta_yaw = self.waypoint[3] -self.yaw                 # nuevo

            rho = sqrt(delta_x*delta_x + delta_y*delta_y +
                       delta_z*delta_z)  # error to the goal
            error_x = math.cos(self.yaw) * delta_x + \
                math.sin(self.yaw) * delta_y
            error_y = -math.sin(self.yaw) * delta_x + \
                math.cos(self.yaw) * delta_y

            # error_x = delta_x
            # error_y = delta_y


            # robot's body frame convertion for x and y linear velocities
            twist.linear.x = max(min(
                self.controlX.calculate_pid(error_x), self.vel_lim), -self.vel_lim)
            twist.linear.y = max(min(
                self.controlY.calculate_pid(error_y), self.vel_lim), -self.vel_lim)
            twist.linear.z = max(min(
                self.controlZ.calculate_pid(delta_z), self.vel_lim), -self.vel_lim)

            twist.angular.z = max(min(
                self.controlZ.calculate_pid(delta_yaw), self.vel_lim), -self.vel_lim) # nuevo
            

            # rospy.loginfo("rho: {:.2f}, yaw: {:.2f},pos = {}, {}, {}, twist: {}".format(rho, self.yaw,
            #                                                                                self.bebopose.position.x, self.bebopose.position.y, self.bebopose.position.z, twist.linear))
            # x_plot['error'].append(error_x)
            # x_plot['time'].append(rospy.get_time())
            # y_plot['error'].append(error_y)
            # y_plot['time'].append(rospy.get_time())
            # z_plot['error'].append(delta_z)
            # z_plot['time'].append(rospy.get_time())
            x_plot.append(error_x)
            y_plot.append(error_y)
            z_plot.append(delta_z)
            time.append(rospy.get_time())
            self.write_and_flush('{}, {}, {}, {}, {}, {}, {}'.format(
                self.bebopose.position.x ,self.bebopose.position.y, self.bebopose.position.z, x_plot, y_plot, z_plot, time))
            self.write_and_flush('\n')
            # https://es.stackoverflow.com/questions/391936/cambiar-columnas-de-str-a-float-leyendo-desde-un-csv

            # rospy.loginfo("rho: {:.2f},pos = {:.2f}, {:.2f}, {:.2f}".format(rho, self.bebopose.position.x, self.bebopose.position.y, self.bebopose.position.z))
            rospy.loginfo("rho: {:.2f},err = {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(rho, error_x, error_y, delta_z,delta_yaw))
            # publishing of angular rates and linear velocities
            self.cmdvel_publisher.publish(twist)
            if rho < 0.1 and not HOVER:
                if len(WAYPOINTS) != 0:
                    self.waypoint = WAYPOINTS.pop(0)
                    print "new waypoint {}, {}, {}".format(
                        self.waypoint[0], self.waypoint[1], self.waypoint[3])
                    rospy.sleep(3)
                else:
                    self.finished = True

            self.rate.sleep()
        self.cmdvel_publisher.publish(twist)
        if self.finished:
            self.cmdvel_publisher.publish(twist)
            self.land()
        print("Move is done!")

    # /bebop/states... callback subscriber to read the state of the parrot
    def update_state(self, message):

        self.state = message.state

    def safety_check(self):
        if self.bebopose.position.x > 4.0 or self.bebopose.position.y > 4.0 or self.bebopose.position.z > 2.0:
            print "EMERGENCY LANDING"
            self.finished = True

    # /vicon/yolanda/yolanda callback subscriber to know the position and the yaw angle of the robot
    # vicon se elimina se cambia por la odometria del drone
    def bebop_pose(self, msg):

        # self.bebopose = msg.transform.translation
        # quat = msg.transform.rotation
        self.bebopose = msg.pose.pose
        quat = self.bebopose.orientation
        (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion(  (quat.x, quat.y, quat.z, quat.w))
        self.safety_check()


    # escribe los valores obtenidos de odometria y contralador PID
    def write_and_flush(self, s):

        self.logfile.write(s)
        self.logfile.flush()


if __name__ == '__main__':
    try:
        node = Bebop_functions()
        node.takeoff()
        rospy.sleep(2)
        node.move()
        node.land()  
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))