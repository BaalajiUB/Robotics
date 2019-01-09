#!/usr/bin/env python
import rosbag
import lab4.msg
from nav_msgs.msg import OccupancyGrid
import rospy
from tf.transformations import euler_from_quaternion
import math

#https://introcs.cs.princeton.edu/python/22module/gaussian.py.html
def gaussian(x, mu=0.0, sigma=1.0):
    x = float(x - mu) / sigma
    return math.exp(-x*x/2.0) / math.sqrt(2.0*math.pi) / sigma

rospy.init_node('map', anonymous=True)
row_size = .20
col_size = .20
ang_size = 30

row = int(7/row_size)
col = int(7/col_size)
ang = int(360/ang_size)

oc_grid_final = [[[0 for k in range(ang)]for j in range(col)]for i in range(row)]
oc_grid_final[11][27][int((200.52-360 + 180)/ang_size)] = 1

threshold = 10**-3 
tags = [[1.25,5.25],[1.25,3.25],[1.25,1.25],[4.25,1.25],[4.25,3.25],[4.25,5.25]]

file = open('/home/first/catkin_ws/src/lab4/trajectory.txt','w')
bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag')
for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
	oc_grid = [[[0 for k in range(ang)]for j in range(col)]for i in range(row)]
	print(msg.timeTag)
	if str(msg._type)=='lab3/Motion':
		(roll,pitch,yaw) = euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w])
		r1 = math.degrees(yaw)
		t = msg.translation
		(roll,pitch,yaw) = euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w])
		r2 = math.degrees(yaw)
		#print(r1,t,r2)
		for i in range(row):
			for j in range(col):
				for k in range(ang):
					s_x,s_y,s_ang = (i*row_size) + row_size/2,(j*col_size) + col_size/2,(k*ang_size) + ang_size/2 - 180 #source cell
					prior = oc_grid_final[i][j][k]
					if(prior>threshold):
						for a in range(row):
							for b in range(col):
								for c in range(ang):
									t_x,t_y,t_ang = (a*row_size) + row_size/2,(b*col_size) + col_size/2,(c*ang_size) + ang_size/2 - 180#target cell
									line_angle = math.degrees(math.atan2(t_y-s_y,t_x-s_x))
									r1_cell = line_angle-s_ang
									if r1_cell>180:
										r1_cell = r1_cell - 360
									elif r1_cell<-180:
										r1_cell = r1_cell + 360
									t_cell = math.sqrt(((t_x-s_x)**2) + ((t_y-s_y)**2))
									r2_cell = t_ang-line_angle
									if r2_cell>180:
										r2_cell = r2_cell - 360
									elif r2_cell<-180:
										r2_cell = r2_cell + 360
									p_r1 = gaussian(r1_cell,r1,ang_size/2)
									p_t = gaussian(t_cell,t,row_size/2)
									p_r2 = gaussian(r2_cell,r2,ang_size/2)
									oc_grid[a][b][c] = oc_grid[a][b][c] + prior*p_r1*p_t*p_r2
		oc_grid_final = oc_grid
	else:
		tot = 0
		tagIndex = msg.tagNum
		t_x,t_y = tags[tagIndex][0],tags[tagIndex][1]
		(roll,pitch,yaw) = euler_from_quaternion([msg.bearing.x,msg.bearing.y,msg.bearing.z,msg.bearing.w])
		r = math.degrees(yaw) 
		t = msg.range
		for i in range(row):
			for j in range(col):
				for k in range(ang):
					s_x,s_y,s_ang = (i*row_size) + row_size/2,(j*col_size) + col_size/2,(k*ang_size) + ang_size/2 - 180 #source cell
					line_angle = math.degrees(math.atan2(t_y-s_y,t_x-s_x))
					r_cell = line_angle - s_ang
					if r_cell>180:
						r_cell = r_cell - 360
					elif r_cell<-180:
						r_cell = r_cell + 360
					t_cell = math.sqrt((t_x-s_x)**2 + (t_y-s_y)**2)
					p_r = gaussian(r_cell,r,ang_size/2)
					p_t = gaussian(t_cell,t,row_size/2)
					oc_grid[i][j][k] = oc_grid_final[i][j][k]*p_r*p_t 
					tot += oc_grid[i][j][k]
		max_cell = -1	
		x_grid,y_grid,ang_grid = -1,-1,-1
		for i in range(row):
			for j in range(col):
				for k in range(ang):
					oc_grid[i][j][k] = oc_grid[i][j][k]/tot 
					if oc_grid[i][j][k]>max_cell:
						max_cell = oc_grid[i][j][k]
						x_grid,y_grid,ang_grid = i,j,k
		print([x_grid+1,y_grid+1,ang_grid+1])
		print(oc_grid[x_grid][y_grid][ang_grid])
		file.write(str(x_grid+1) + ',' + str(y_grid+1) + ',' + str(ang_grid+1) + '\n')
		oc_grid_final = oc_grid
file.close()
bag.close()
												

