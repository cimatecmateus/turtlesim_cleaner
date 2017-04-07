#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def print_poser():
    #Creating our node
    rospy.init_node('poser_subscriber', anonymous = True)
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    #Instance of class Pose for read data from /turtle1/pose topic
    #self.pose = Pose()
    rospy.spin()
    
    #Callback function to read data from /turtle1/Pose topic
def callback(data):
    print data
    print "\n"


if __name__ == '__main__':
    try:
        print_poser()
    except rospy.ROSInterruptException: pass


