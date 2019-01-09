#!/usr/bin/env python
import rospy
import numpy as np
from numpy import linalg as LA
import random
from sensor_msgs.msg import LaserScan
#from sensor_msgs.msg import PointCloud2
#import sensor_msgs.point_cloud2 as pc2 
#not pointcloud2.It has no read_points() function. What is pointcloud2 then?
from laser_geometry import LaserProjection
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

laser_proj = LaserProjection()
#pub = rospy.Publisher('/pointcloud_gen', PointCloud2, queue_size=1)
pub_line_min_dist = rospy.Publisher('/line_conn', Marker, queue_size=10) #queue_size must be 1 to store only latest data
class lines:
    def __init__(self, p1, p2, c=0):
        self.p1 = p1
        self.p2 = p2
        self.c  = c

    def __str__(self):
        return str(self.p1) + ',' + str(self.p2) + ' -> ' + str(self.c)
    
class points_2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x) + ',' + str(self.y)

#https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
def find_dist(p1,p2,p3):
    d = np.abs(np.cross([p2.y-p1.y,p2.x-p1.x],[p1.y-p3.y,p1.x-p3.x])) / LA.norm([p2.x-p1.x,p2.y-p1.y]) 
    return d  #norm = rms of args

threshold = 0.02
K=10

def ransac(pts):

	final_list = []

	#while there are more than 10 points left
	while(len(pts)>10):
	    print(len(pts))
	    #run for k times
	    pt_ind_list = []
	    line_list = []
	    for i in range(K):
		p = random.sample(range(len(pts)),2) #2 indices without repetation
		while p in pt_ind_list:
		    p = random.sample(range(len(pts)),2) #2 indices without repetation

		#local 2 points selected
		pt_ind_list.append(p)

		p1 = pts[p[0]]
		p2 = pts[p[1]]

		line_list.append(lines(p1,p2))

		cnt=0
		for i in pts:
		    if not(i.x==p1.x and i.y==p1.y) and not(i.x==p2.x and i.y==p2.y):
		        p3 = i
		        dist = find_dist(p1,p2,p3)
		        if dist<=threshold and dist>= (-1 * threshold):
		            cnt=cnt+1
		#            print(p3 , ' = ' , str(dist) , ' -> ' , str(cnt))
		        else:
		#            print(p3 , ' = ' , str(dist))
		             pass
		line_list[-1].c = cnt
		#print(line_list[-1])

	    mini=-1
	    mini_ind=-1
	    for i in range(len(line_list)):
		if line_list[i].c> mini:
		    mini = line_list[i].c
		    mini_ind = i
	    print(mini,mini_ind)
	    
	    final_list.append(line_list[mini_ind])

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
		#print('removing',i)

	    print(len(pts))
	    #print()

	print(final_list)
	return final_list

#https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/
def make_marker_line(line_list):
    marker = Marker()
    marker.header.frame_id = "/base_laser_link" #was /odom, Now same as laser's frame but invariant
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03 #was 0.03
    marker.scale.y = 0.00 #was 0.03 #must not be used according to docs
    marker.scale.z = 0.00 #was 0.03 #must not be used according to docs

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0 #was 0.0. It is to make it visible

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0 #was 1.0. Trying to allign marker with laserscan

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    for line in line_list:
	    # first point
	    first_line_point = Point()
	    first_line_point.x = line.p1.x
	    first_line_point.y = line.p1.y
	    first_line_point.z = 0.0
	    marker.points.append(first_line_point)
	    # second point
	    second_line_point = Point()
	    second_line_point.x = line.p2.x
	    second_line_point.y = line.p2.y
	    second_line_point.z = 0.0
	    marker.points.append(second_line_point)

    # Publish the Marker
    pub_line_min_dist.publish(marker)	

flg=0
def callback(scan):
	global flg
	#pc = laser_proj.projectLaser(scan)
	#pub.publish(pc) #to display in rviz
	#pc2pts = pc2.read_points(pc) #why read_points_list() not available?
	if flg==0:
		pts = []
		#avg = [0,0]	
		#cnt=0
		'''	
		for p in pc2pts:
			tmp = points(p[0],p[1])
			pts.append(tmp)
			#avg[0]+=p[0]
			#avg[1]+=p[1]
			#cnt+=1
		'''
		r= scan.angle_min	
		#r=0
		inc = scan.angle_increment 
		for p in range(len(scan.ranges)):
			if scan.ranges[p]<3:
				x = scan.ranges[p] * math.cos(r)
				y = scan.ranges[p] * math.sin(r) 
				tmp = points_2D(x,y)
				pts.append(tmp)
			r+= inc
			print(r)

		line_list = ransac(pts)
		#print(avg[0]/cnt,avg[1]/cnt)
		#for line in line_list:
		make_marker_line(line_list)		
		flg=1
	else:
		flg= (flg+1)%5	#skipping 4 out of every 5 frames
	
if __name__ == '__main__':
    try:
	rospy.init_node('ransac', anonymous=True)    
		
	sub=rospy.Subscriber('/base_scan',LaserScan,callback)	
	rospy.spin() #Keeps the node alive till Ctrl+C is entered to kill the process
    except rospy.ROSInterruptException:
        pass	
