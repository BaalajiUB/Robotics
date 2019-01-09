#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #publisher(topic,message type,queue_size =1 #only latest is stored)

#function publishes a twist msg to roatate at random velocity between -90 to 90 along Z-axis ie.. about the position clockwise or anti-clockwise
def rotate():   
    twist=Twist() #Twist msg object
    #print("rotate")
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;             
    twist.angular.x = 0;
    twist.angular.y = 0;   
    twist.angular.z = random.randint(-90,90);
    pub.publish(twist)

#function publishes a twist msg to move straight at 2m/s along x-axis
def straight():
    #print("straight")
    twist=Twist() #Twist msg object
    twist.linear.x = 2;
    twist.linear.y = 0;
    twist.linear.z = 0;             
    twist.angular.x = 0;
    twist.angular.y = 0;   
    twist.angular.z = 0;  
    pub.publish(twist)

def callback(data):   
   #must make sure that the execution time of this module is less than the subscription rate
   #else there is mismatch between data received and published
   #must launch along with world to synchrnize laser data with the bot 
   #working properly after adding node to launch file
   #instructions updated at rate of ~Hz
   twist=Twist()
   mini = 1
   for i in range(0,360):    
       if data.ranges[i]< 1:
	    mini = data.ranges[i]
	    #rotate() #If anything withing range of 1m is found, the robot turns. It is to avoid all possible outputs
	    break
   if mini==1: #If nothing is found near 1m, moves straight
       #straight()
       pass
       	

if __name__ == '__main__':
    try:
	rospy.init_node('evador', anonymous=True)    
		
#	while not rospy.is_shutdown():
	sub=rospy.Subscriber('/base_scan',LaserScan,callback) #invokes everytime a new msg is posted in base_scan topic
	rospy.spin() #Keeps the node alive till Ctrl+C is entered to kill the process
    except rospy.ROSInterruptException:
        pass	

