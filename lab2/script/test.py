#!/usr/bin/env python
import rospy
import math
import numpy as np
from numpy import linalg as LA
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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

tan = math.degrees(math.atan(11/12.5))
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

#ransac
threshold=0.02
K=10
c=0 #first turn on seeing obstacle as 90 degrees

target_rotation=1 #Not 0. It is to maintain the flow of the algorithm

#on_line
on_l = 0
t_x=t_y=0

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
    for x in range(2):
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
    if bot_y >= 8.5 and bot_y <= 9.5 and bot_x >= 4 and bot_x<=5: #+/-0.5
        return True
    return False

def on_line():
    global bot_x,bot_y
    p3 = points_2D(bot_x,bot_y)
    p1 = points_2D(4.5,9)
    p2 = points_2D(-8,-2)	
    dist = find_dist(p1,p2,p3)	
    if dist<=0.75:
        return True
    return False

def straight():
    twist = Twist()
    twist.linear.x = 10
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    
    pub.publish(twist)
    
def rotate(angle):
    speed = angle*0.01
    #if angle<0:
    #    speed = -speed
    
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = speed
    
    pub.publish(twist)

def laser_callback(laser):
    global line,ent,wall,curr_orientation,flg,target_rotation,r2,c

    if on_goal_pos():
        return

    if line==1 and r1==0:
        print('curr_orientation:   ',curr_orientation)
        print('target_orientation: ',tan)
        angle = tan - curr_orientation
        print('rotation_angle:     ',angle)
        
        if angle>0.5 or angle<-0.5: 
            rotate_1()
            return

        dist=0.5 #obstacle distance #done
        mini=dist
        no_obstacle=False
	
        for i in range(160,201):
            if laser.ranges[i]<mini:
                mini=laser.ranges[i]
        if mini==dist:
            no_obstacle=True

        print('No Obstacle', no_obstacle)
        if no_obstacle: 
            straight()
	    print("straight")

        else:
            line=0
            wall=1
	    ent=0
	    r2=1
	    

    if wall == 1 and flg == 0 and r2==1:
        pts=[]
        r = laser.angle_min
        inc = laser.angle_increment

        for p in range(len(laser.ranges)):
            if laser.ranges[p]<1.5:
                x = laser.ranges[p] * math.cos(r)
                y = laser.ranges[p] * math.sin(r)
                tmp = points_2D(x,y)
                pts.append(tmp)
            r+=inc
	
	if len(pts)<5:
		target_rotation = curr_orientation + 90 #turns left
		print('corner straight')
		straight()
		straight()

	else:
		best_line = ransac(pts)
		
		x1 = best_line.p1.x
		y1 = best_line.p1.y
		x2 = best_line.p2.x
		y2 = best_line.p2.y

		if x1==x2:
		    target_rotation = curr_orientation + 0
		    straight()

		elif y1==y2:
		    target_rotation = curr_orientation + 90 #always rotates to right
		    straight()
		else:
		    angle = math.degrees(math.atan((y2-y1)/(x2-x1)))
		    print(angle)
		    #if angle<0:
			#angle = angle + 180
			#print(angle)
		    #angle = angle - 90  
		    #print(angle)
		    print(c)
	     	    if c==0:
		    	angle = -90 #rotates right always
			c=1
			#print(angle)
		    target_rotation = curr_orientation + angle
		    
		#print('ransac: ',curr_orientation,angle,target_rotation)
		#print(y2,y1,x2,x1)
        #flg=1
	ent=1
	r2=0
        

def odom_callback(odom):
    global line,r1,r2,wall,ent,bot_x,bot_y,curr_orientation,target_rotation,flg,c,on_l,t_x,t_y

    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y

    ori = odom.pose.pose.orientation
    ori_l = [ori.x,ori.y,ori.z,ori.w]
    (r, p, y) = euler_from_quaternion(ori_l)		

    curr_orientation = math.degrees(y)	#(math.degrees(math.acos(odom.pose.pose.orientation.w))*2)+90

    #WALL_FOLLOW to GOAL_SEEK
    if on_line() and wall==1:
	if on_l==0:
		t_x = bot_x
		t_y = bot_y
		on_l=1
	elif on_l==1 and (bot_x>t_x+1 or bot_x<t_x-1) and (bot_y>t_y+1 or bot_y<t_y-1):
		print('on_line')
		wall=0
		line=1
		c=0
		on_l=0
		return

    #rotation towards goal_line    
    if line==1 and r1==1: 
        #ent=0
        #logic to rotate to goal angle #done
        angle = tan - curr_orientation
        if angle<0.5 and angle>-0.5: 
            #ent=1
            r1=0
        else:
            rotate(angle)

    #move along ransac_line
    if wall==1 and ent==1:
        angle_1 = target_rotation - curr_orientation
	#print(angle_1,'4')
        if angle_1>0.5 or angle_1<-0.5: #compute
            ent=0
	    flg=1
            rotate_2()
            return
	else:
	    ent=0
	    r2=1
	    print("straight")    
            straight()

    #rotation towards ransac_line 
    if wall==1 and r2==1 and flg==1:
        #ent=0
        #logic to rotate to ransac angle #done
        angle_1 = target_rotation - curr_orientation
	#print(angle_1)
        if angle_1<0.5 and angle_1>-0.5: 
	    #print("straight")
	    #straight()
            #ent=1
            #r2=0
	    flg=0
	    print("straight")
	    straight()
        else:
            rotate(angle_1)
	    #print(target_rotation,curr_orientation,angle_1)


if __name__ == '__main__':
    try:
	rospy.init_node('bug2', anonymous=True)    
		
	sub=rospy.Subscriber('/base_scan',LaserScan,laser_callback)
	sub1 = rospy.Subscriber('/base_pose_ground_truth',Odometry,odom_callback) 
	rospy.spin() 
    except rospy.ROSInterruptException:
        pass	




