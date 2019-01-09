#!/usr/bin/env python
import rospy
import math
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Pose:
	def __init__(self,x=0,y=0,theta=0):
		self.x = x
		self.y = y	
		self.theta = theta

class Bot:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/base_pose_ground_truth',Odometry, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
		#print('Prev pose: ',self.pose.x,self.pose.y)
		self.pose.x = round(data.pose.pose.position.x, 4)
		self.pose.y = round(data.pose.pose.position.y, 4)
		self.pose.theta = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
		#print('Curr pose: ',self.pose.x,self.pose.y)

    def euclidean_distance(self, goal_pose):
        return math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) + math.pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return self.euclidean_distance(goal_pose)  * constant

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return (self.steering_angle(goal_pose) - self.pose.theta) * constant 

    def move2goal(self,x,y):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y	

        distance_tolerance =  0.3 #input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()


class node:
    def __init__(self,parent,cor):
        self.parent = parent
        self.cor = cor
        self.g = sys.maxsize
        self.h = sys.maxsize
        self.f = sys.maxsize

def validate_cell(map_grid,x,y,rows,cols): #true if invalid
    try:
        op = (map_grid[x][y]=='1' or x<0 or x>=rows or y<0 or y>=cols)
        return op
    except:
        return True

def generate_grid():
    #Reading map from text file into a matrix 20X18 #0 & 1 are strings
    map_read = open('/home/first/catkin_ws/src/lab5/script/map.txt','r')
    lines = map_read.readlines()
    map_read.close()

    map_grid = []
    for line in lines:
        t = ''
        for i in line:
            if i.isdigit():
              t+=i
        map_grid.append(list(t))
    #map_grid = map_grid[::-1]
    for i in map_grid:
        print(''.join(i))
    ##90 degress clockwise rotation##
    new = [[0 for j in range(len(map_grid))] for i in range(len(map_grid[0]))]
    for i in range(len(new)):
	for j in range(len(new[0])):
	    new[i][j] = map_grid[len(new[0])-j-1][i]
    return new

def main(map_grid):
    rows = len(map_grid)
    cols = len(map_grid[0])

    start = node(None,(-8-offset_x-1,-2-offset_y-1))
    start.g = start.h = start.f = 0
    end = node(None,((args[0]//1)-offset_x-1,(args[1]//1)-offset_y-1))
    end.g = end.h = end.f = 0

    print(start.cor)
    print(end.cor)
   
    if validate_cell(map_grid,start.cor[0]//1,start.cor[1]//1,rows,cols):
        print("Invalid start point")
        return 0

    if validate_cell(map_grid,int(end.cor[0]),int(end.cor[1]),rows,cols):
        print("Invalid end point")
        return 0
   
    open_list = []
    close_list = []

    open_list.append(start)
    loop_no=0
    while(len(open_list)>0):
	print("open_list")
        for i in open_list:
            print(i.cor,i.f)
        print(len(open_list))
        loop_no+=1
        #find node with min f
        t = open_list[0]
        for i in open_list:
            if i.f<t.f:
                t = i
        open_list.remove(t)
        close_list.append(t)

        #if t is end node, return
        if t.cor==end.cor:
            return t
        #append all neighbors to the open_list if valid
        print("Chosen Node: ",t.cor,t.f)
	print("-----------------------------------------------------------------")
        for i in neighbors:
            x = int(t.cor[0]+i[0])
            y = int(t.cor[1]+i[1])
            if validate_cell(map_grid,x,y,rows,cols)==False:
                #print(map_grid[x][y],validate_cell(map_grid,x,y,rows,cols),(x,y))
                #if in close_list: skip
                f=0
                for j in close_list:
                    if j.cor == (x,y):
                        f=1
                        break
                if f==1:
                    #print('In close_list')
                    continue
                #update if in open_list
                e_node = None
                for j in open_list:
                    if j.cor == (x,y):
                        e_node = j
                        break
                   
                if e_node==None:
                    nd = node(t,(x,y))
                    nd.g = t.g + math.sqrt((i[0]**2) + (i[1]**2))
                    nd.h = math.sqrt((end.cor[0]-x)**2 + (end.cor[1]-y)**2)
                    nd.f = nd.g + nd.h
                    open_list.append(nd)
                    #print('Not in open_list', nd.f)

                else:
                    if t.g+1 < e_node.g:
                        e_node.g = t.g + math.sqrt((i[0]**2) + (i[1]**2))
                        e_node.f = e_node.g + e_node.h
                        e_node.parent = t
                        #print('Updated in open_list',e_node.f)
        #if loop_no==3:
            #break
    return 1

def print_path(tail):
    for i in map_grid:
        print(''.join(i))
    path = []
    obs = []
    while(tail):
        path.append((tail.cor[0]+offset_x+1,tail.cor[1]+offset_y+1))
        map_grid[tail.cor[0]][tail.cor[1]]='X'
        tail = tail.parent
    path = path[::-1]
    print(path)
    for i in map_grid:
        print(''.join(i))
    return path
   
if __name__ == '__main__':
	try:
		rospy.init_node('path', anonymous=True)
		arg0 = float(rospy.get_param('~goalx'))
		arg1 = float(rospy.get_param('~goaly'))
		#args = [4.5,9.0]#input from prompt
		args = [arg0,arg1]
		neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
		map_grid = generate_grid()
		for i in map_grid:
		    print(''.join(i))
		#exit(0)
		offset_x = - len(map_grid)//2 -1
		offset_y = - len(map_grid[0])//2 - 1
		tail = main(map_grid)
		if tail==1:
		    print('No path available')
		    exit(0)
		elif tail!=0:
		    path = print_path(tail)
		print(path[1][0],path[1][1])
		Bot = Bot()
		for point in path:
			Bot.move2goal(point[0],point[1])
		exit(0)			
	except rospy.ROSInterruptException:
		print("ROSInterruptException")	

