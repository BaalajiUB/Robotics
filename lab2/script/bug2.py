#!/usr/bin/env python
import rospy
import math
import numpy as np
from numpy import linalg as LA
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

#flow control
line=1
wall=0
ent=1
r1=0
r2=0

#odom data
bot_x=0
bot_y=0
curr_orientation=1

#ransac data
flg=0

tan = 180 - math.degrees(math.atan(11/12.5))
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

#ransac
threshold=0.02
K=10

target_rotation=0

class lines:
    def __init__(self,p1,p2,c=0):
        self.p1 = p1
        self.p2 = p2
        self.c = c

        def __str__(self):
            return str(self.p1) + ',' + str(self.p2) + '->' + str(self.c)

class points_2D:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x) + ',' + str(self.y)

def find_dist(p1,p2,p3):
    d = np.abs(np.cross([p2.y-p1.y,p2.x-p1.x],[p1.y-p3.y,p1.x-p3.x]))/LA.norm([p2.x-p1.x,p2.y-p1.y])
    return d

def ransac(pts):

    final_list = []

#    while(len(pts)>10):
    for x in range(10):
          pt_ind_list = []
          line_list = []

          for i in range(K):
              p = random.sample(range(len(pts)),2)
              while p in pt_ind_list:
                  p = random.sample(range(len(pts)),2)

              p1 = pts[p[0]]
              p2 = pts[p[1]]

              line_list.append(lines(p1,p2))

              cnt=0
              for i in pts:
                  if not(i.x==p1.x and i.y==p1.y) and not(i.x==p2.x and i.y==p2.y):
                      p3 = i
                      dist = find_dist(p1,p2,p3)
                      if dist<=threshold and dist>= (-1 * threshold):
                          cnt = cnt+1
                      else:
                          pass
              line_list[-1].c = cnt

          mini = -1
          mini_ind = -1
          for i in range(len(line_list)):
              if line_list[i].c>mini:
                  mini = line_list[i].c
                  mini_ind = i

          final_list.append(line_list[mini_ind])
          x = x+1  
          #delete points logic
          '''
          p1 = line_list[mini_ind].p1
          p2 = line_list[mini_ind].p2

          pts_new = []
          pts_new.append(p1)
          pts_new.append(p2)

          for i in pts:
              if not(i.x==p1.x and i.y==p1.y) and not(i.x==p2.x and i.y==p2.y):
                  p3 = i
                  dist = find_dist(p1,p2,p3)
                  if dist<=threshold and dist>= (-1 * threshold):
                      pts_new.append(i)

          for i in pts_new:
              pts.remove(i)
          '''
          #end delete point logic
    mini=-1
    mini_ind = -1

    for i in range(len(final_list)):
        if final_list[i].c > mini:
            mini=final_list[i].c
            mini_ind= i
            
    return final_list[mini_ind]

def rotate_1():
    global r1
    r1=1

def rotate_2():
    global r2
    r2=1


def on_goal_pos():
    if bot_y == 9 and bot_x == 4.5:
        return True
    return False

def on_line():
    if ((bot_y - 2)/(bot_x-8)) == (11/12.5):
        return True
    return False

def straight():
    twist = Twist()
    twist.linear.x = 0.7
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    
    pub.publish(twist)
    
def rotate(angle):
    speed = angle*0.01
    if angle<0:
        speed = -speed
    
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = speed
    
    pub.publish(twist)

def stop():
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    
    pub.publish(twist)

def laser_callback(laser):
    global line,ent,wall,curr_orientation,flg,target_rotation
    if on_goal_pos():
        return
    if line==1 and ent==1:
        #print('curr_orientation:   ',curr_orientation)
        #print('target_orientation: ',tan)
        angle = tan - curr_orientation
        #print('rotation_angle:     ',angle)
        
        if angle>0.0001: 
            rotate_1()
            return

        dist=1 #obstacle distance #done 
        mini=dist

        no_obstacle=False

        #for i in range(len(laser.ranges)):
	for i in range(90,271):
            if laser.ranges[i]<mini:
                mini=laser.ranges[i]
        if mini==dist:
            no_obstacle=True
                
	print('No Obstacle: ',no_obstacle)
        if no_obstacle: 
            straight()
        else:
            line=0
            wall=1
	    stop()

    if wall == 1 and flg == 0 and r2==0:
	#flg=1        
	pts=[]
        r = laser.angle_min
        inc = laser.angle_increment

        for p in range(len(laser.ranges)):
            if laser.ranges[p]<2:
                x = laser.ranges[p] * math.cos(r)
                y = laser.ranges[p] * math.sin(r)
                tmp = points_2D(x,y)
                pts.append(tmp)
            r+=inc
        best_line = ransac(pts)
	#print("ransac computed")
	x1 = best_line.p1.x 
	y1 = best_line.p1.y 
	x2 = best_line.p2.x
	y2 = best_line.p2.y        
	
        if x1==x2:
            target_rotation=curr_orientation + 0
        elif y1==y2:
	    if y1>0:
            	target_rotation=curr_orientation + 90
	    else:
		target_rotation=curr_orientation - 90
        else:
            slope_angle=0
            if x1>x2:
                slope_angle = math.degrees(math.atan((y1-y2)/(x1-x2)))
            elif x1<x2:
                slope_angle = math.degrees(math.atan((y2-y1)/(x2-x1)))
                
            if slope_angle>0:
                angle = curr_orientation + 180 - slope_angle
            else:
                angle = curr_orientation + 270 + slope_angle #as slope_angle is negative
                
            target_rotation = angle
            #print(ent,wall)
        #tf of line to global frame
        #find slope of line in global frame
        #reassign global variable that stores the frame
        flg=0
    #elif wall==1 and flg!=0:
    #    flg = (flg+1)%5 #skipping 4 out of 5 frames
        
prev_orientation = curr_orientation

def odom_callback(odom):
    global line,r1,r2,wall,ent,bot_x,bot_y,curr_orientation,target_rotation,prev_orientation

    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y
    curr_orientation = (math.degrees(math.acos(odom.pose.pose.orientation.w))*2)+90
    print('odom_curr_orientation',curr_orientation)
    #rotation towards goal_line    
    if line==1 and r1==1: 
        ent=0
        #logic to rotate to goal angle #done
        angle = tan - curr_orientation
        if angle<0.0001: 
            ent=1
            r1=0
	    print("r1 completed")
        else:
            rotate(angle)
	    print('angle',angle)

    #move along ransac_line
    if wall==1 and ent==1:
        angle_1 = target_rotation - curr_orientation - (curr_orientation - prev_orientation)  #as target_rotation dependant on curr_orientation, 2*
	#angle_1 = prev_orientation - curr_orientation	
	#print(target_rotation,curr_orientation,angle_1)	
	#print(angle_1,'calling r2')
	print('angle_1',angle_1)
	if r2==0:
	    prev_orientation = curr_orientation
	    #prev_orientation = target_rotation		
        if angle_1>0.5: 
            rotate_2()
	    #ent=0
            return
	#else:
	    #print("straight")
            #straight()
	    #stop()
	    #rospy.sleep(0.1)
	    #print("straight and stop")


    #rotation towards ransac_line 
    if wall==1 and r2==1:
        ent=0
        #logic to rotate to ransac angle #done
        angle_1 = target_rotation - curr_orientation - (curr_orientation - prev_orientation) #not updating properly?? 
	print(angle_1,"r2")
        if angle_1<=0.5: 
	    prev_orientation = curr_orientation
	    print("r2 completed")
            straight()
	    print("straight")
            ent=1
            r2=0

        else:
            rotate(angle_1)
	    print("rotating ", angle_1)

    if on_line() and wall==1:
        wall=0
        line=1


if __name__ == '__main__':
    try:
	rospy.init_node('bug2', anonymous=True)    
		
	sub=rospy.Subscriber('/base_scan',LaserScan,laser_callback) #invokes everytime a new msg is posted in base_scan topic
	sub1 = rospy.Subscriber('/base_pose_ground_truth',Odometry,odom_callback) #holds absolute truth position
	rospy.spin() #Keeps the node alive till Ctrl+C is entered to kill the process
    except rospy.ROSInterruptException:
        pass	

