import random
import numpy as np
from numpy import linalg as LA

class lines:
    def __init__(self, p1, p2, c=0):
        self.p1 = p1
        self.p2 = p2
        self.c  = c

    def __str__(self):
        return str(self.p1) + ',' + str(self.p2) + ' -> ' + str(self.c)
    

class points:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x) + ',' + str(self.y)

def points_gen():
    pts = []
    for i in range(10):
        for j in range(10):
            x=random.randint(1,200)
            y=random.randint(1,200)
            f = 0
            for k in pts:
                if k.x==x and k.y==y:
                    f = 1
                    break
            if (f==0):
                new_p = points(x,y)
                pts.append(new_p)

    #print(len(points['x']))
    return pts

def find_dist(p1,p2,p3):
    d = np.abs(np.cross([p2.y-p1.y,p2.x-p1.x],[p1.y-p3.y,p1.x-p3.x])) / LA.norm([p2.x-p1.x,p2.y-p1.y]) #https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
    return d  #norm = rms of args

#print (find_dist(points(0,0),points(2,2),points(0,1)))


#Total points available
threshold = 20
pts = points_gen()
print(len(pts))
#pt_ind_list = []

K=10

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

        #for i in p:
        #    print(pts[i])

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
    
    final_list.append(str(line_list[mini_ind]))

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
        print('removing',i)

    print(len(pts))
    print()

print(final_list)
