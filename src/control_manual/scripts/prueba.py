#!/usr/bin/env python
import rospy

from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged

global previo 
previo = 0

def callback_function(speed):
    global previo
    # velocidad = speed.speedX
    actual = rospy.get_time()
    # rospy.loginfo("Vel X: %s ", velocidad)
    dt = actual - previo
    print(dt)
    previo = actual


def publisher():
  
    rospy.init_node('Bebop_velocidad', anonymous=True)
    rospy.Subscriber('/bebop/states/ardrone3/PilotingState/SpeedChanged',
                                                        Ardrone3PilotingStateSpeedChanged, callback_function)
    rospy.spin()


if __name__ == '__main__':
    publisher()
    
