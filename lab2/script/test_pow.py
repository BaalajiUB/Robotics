#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

r=p=y=0.0

def odom_callback(odom):
	global r,p,y
	ori = odom.pose.pose.orientation
	ori_l = [ori.x,ori.y,ori.z,ori.w]
	(r, p, y) = euler_from_quaternion(ori_l)		
	print(y)
	print(math.degrees(y))

if __name__ == '__main__':
    try:
	rospy.init_node('bug2', anonymous=True)    
	sub1 = rospy.Subscriber('/base_pose_ground_truth',Odometry,odom_callback) 
	rospy.spin() 
    except rospy.ROSInterruptException:
        pass	



