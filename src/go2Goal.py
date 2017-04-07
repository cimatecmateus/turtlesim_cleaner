#!/usr/bin/env python

import sys
import os
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
        #Instance of class Pose for read data from /turtle1/pose topic
        self.pose = Pose()
        #Instance of class Twist for publish in the /turtle1/cmd_vel topic
        self.vel_msg = Twist()
        #Instance of class Pose for user inputs
        while self.pose.x == 0:
            pass
        self.user_pose = Pose()
        self.user_pose.x = self.pose.x
        self.user_pose.y = self.pose.y
        print self.user_pose
        #Define frequency loop to 65Hz
        self.rate = rospy.Rate(65)
    
    #Callback function to read data from /turtle1/Pose topic
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    #Function to receiver the data from user input
    def userInput(self):
        #Old point of user
        self.old_pose_y = self.user_pose.y
        #User inputs
        self.user_pose.x = input("Set your x goal: ")
        self.user_pose.y = input("Set your y goal: ")
        self.distance_tolerance = input("Set your tolerance: ")

    def rotate(self, ref_vector, comp_vector):
        #Calculates scalar product
        dot_product = np.dot(ref_vector, comp_vector)
        #Calculates vector module
        ref_vector_abs = np.sqrt((ref_vector*ref_vector).sum())
        comp_vector_abs = np.sqrt((comp_vector*comp_vector).sum())
        #Calculates the cosseno of angle between the vectors
        cos_angle = (dot_product / (ref_vector_abs * comp_vector_abs))
        #Calculates de angle between vectors
        angle = math.acos(cos_angle)

        #Angular velocity at degrees per second
        angular_vel = 40

        #if y of user point above the actual point, rotate in the counterclockwise direction
        if(self.user_pose.y >= self.old_pose_y):
            angular_speed = (angular_vel * 2 * math.pi) / 360
        #Else, rotate in the clockwise direction
        else:
            angular_speed = -(angular_vel * 2 * math.pi) / 360

        #We wont use linear components
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        #Set the angular velocity
        self.vel_msg.angular.z = angular_speed
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        #Rotate to direction desired
        while(abs(current_angle) < angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            self.rate.sleep()

        #Forcing the turtle to stop
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def move2point(self):
        while math.sqrt(math.pow((self.user_pose.x - self.pose.x), 2) + math.pow((self.user_pose.y - self.pose.y), 2)) >= self.distance_tolerance:

            #Porportional Controller linear velocity in the x-axis:
            self.vel_msg.linear.x = 1.5 * math.sqrt(math.pow((self.user_pose.x - self.pose.x), 2) + math.pow((self.user_pose.y - self.pose.y), 2))
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0

            #Publishing our vel_msg
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)
        #rospy.spin()

    def run(self):
        #Initial vector of direction reference
        ref_vector = np.array([1,0])
        response = "yes"
        while response == "yes":
            os.system("clear")
            #Take the parameters os user
            self.userInput()
            #Vector that point to direction
            dir_vector = np.array([self.user_pose.x - self.pose.x, self.user_pose.y - self.pose.y])
            #Rotate to direction specified
            self.rotate(ref_vector,dir_vector)
            #Move to point specified for user
            self.move2point()
            #Change the vector direction
            ref_vector = dir_vector
            print "\nMove to new point? yes or not?"
            response = raw_input("Response: ")
        print "\nPlease, press crtl+c to exit"
        rospy.spin()


if __name__ == '__main__':
    try:
        #Instance of turtle
        indigo = turtle()
        #Run movement
        indigo.run()

    except rospy.ROSInterruptException: pass


