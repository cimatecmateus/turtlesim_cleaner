#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtle():
    def __init__(self):
        #Creating our node
        rospy.init_node('turtlesim_controller', anonymous = True)
        #Creating our publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        #Creating our subscriber
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        #Instance of class Pose
        self.pose = Pose()
        #Define frequency loop to 10Hz
        self.rate = rospy.Rate(10)
    
    #Callback function to read data from /turtle1/Pose topic
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    #Function to receiver the data from user input
    def userInput(self):
        #Instance of class Pose
        self.user_pose = Pose()
        #User inputs
        self.user_pose.x = input("Set your x goal: ")
        self.user_pose.y = input("Set your y goal: ")
        self.distance_tolerance = input("Set your tolerance: ")
        #Instance of class Twist
        self.vel_msg = Twist()

    def rotate(self):
        ref_vector = np.array([1,0])
        comp_vector = np.array([self.user_pose.x - self.pose.x, self.user_pose.y - self.pose.y])
        dot_product = np.dot(ref_vector, comp_vector)
        ref_vector_abs = np.sqrt((ref_vector*ref_vector).sum())
        comp_vector_abs = np.sqrt((comp_vector*comp_vector).sum())
        cos_angle = (dot_product / (ref_vector_abs * comp_vector_abs))
        angle = math.acos(cos_angle)

        angular_speed = 10*2*math.pi/360

        #We wont use linear components
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

        self.vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            self.rate.sleep()
            print current_angle

        #Forcing our robot to stop
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        rospy.spin()


if __name__ == '__main__':
    try:
        #Testing our function
        x = turtle()
        x.userInput()
        x.rotate()

    except rospy.ROSInterruptException: pass


