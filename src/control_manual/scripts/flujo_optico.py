#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from nav_msgs.msg import Odometry 
import numpy as np

from lyapunov import *


class Bebop_functions():
    def __init__(self):
        # Bebop_control node creation
        rospy.init_node('Bebop_control', anonymous=True)

        # Publisher para despegue
        self.takeoff_publisher = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)

        # Publisher para aterrizaje
        self.landing_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        # Publisher comando de velocidad
        self.cmdvel_publisher = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/bebop/states/ardrone3/PilotingState/FlyingStateChanged'. self.update_state is called
        # when a message of type Ardrone3PilotingStateFlyingStateChanged is received.
        self.pilotingstate_subscriber = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
                                                         Ardrone3PilotingStateFlyingStateChanged, self.update_state)

        self.bebopcoord_subscriber = rospy.Subscriber("/bebop/odom", Odometry, self.comandos_velocidad)

        # self.bebopvel_subscriber = rospy.Subscriber('bebop/states/ARDrone3/PilotingState/SpeedChanged', Ardrone3PilotingStateSpeedChanged)

        self.state = None
        self.rate = rospy.Rate(10)



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



    # /bebop/states... callback subscriber to read the state of the parrot
    def update_state(self, message):
        self.state = message.state


    def safety_check(self):
        if self.bebopose.position.x > 8.0 or self.bebopose.position.y > 8.0 or self.bebopose.position.z > 3.0:
            print "EMERGENCY LANDING"
            self.finished = True
            self.finished_yaw = True

   
    # def bebop_pose(self, msg):

    #     # self.bebopose = msg.transform.translation
    #     # quat = msg.transform.rotation
    #     self.bebopose = msg.pose.pose
    #     quat = self.bebopose.orientation
    #     (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion(
    #         (quat.x, quat.y, quat.z, quat.w))
    #     self.safety_check()


    def comandos_velocidad(self, msg):

        self.bebopose = msg.pose.pose
        quat = self.bebopose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

        posx = self.bebopose.position.x
        posy = self.bebopose.position.y
        posz = self.bebopose.position.z
        # posyaw = self.yaw            

        # t = rospy.Time.now()
        # ts = (t - self.t_old).to_sec()
        
        LYAPUNOV.set_posicion(posx, posy, posz)
        self.hx, self.hy, self.hz, self.hyaw = LYAPUNOV.compute(ts)

    
        twist = Twist()
        twist.linear.x = max(min(self.hx, 1), -1)
        twist.linear.y = max(min(self.hy, 1), -1)
        twist.linear.z = max(min(self.hz, 1), -1)
        twist.angular.z = max(min(self.hyaw, 1), -1)
        self.cmdvel_publisher.publish(twist)


if __name__ == '__main__':
    try:
        node = Bebop_functions()
        # node.waypoint()
        node.takeoff()
        rospy.sleep(2)
        node.comandos_velocidad(msg)
      
        node.land()  
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))