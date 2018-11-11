import math
#https://stackoverflow.com/questions/12412895/calculate-probability-in-normal-distribution-given-mean-std-in-python #3rd answer
def normpdf(x, mean, sd):
    var = float(sd)**2
    pi = 3.1415926
    denom = (2*pi*var)**.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

#Gaussian y-value logic
#for x,mean,sd in [(-1,0,1),(0,0,1),(1,0,1)]:
    #print(normpdf(x,mean,sd))

x_cell = 700//10
y_cell = 700//10
deg_cell = 360//10

x,y,deg = x_cell*11 + x_cell//2,y_cell*27 + y_cell//2,200.52 #mid-point of cell 12 along x and 28 along y 

oc_grid = [[[0 for k in range(deg_cell)]for j in range(y_cell)]for i in range(x_cell)]
oc_grid[12-1][28-1][(int(200.52)//10)] = 1

#rotation
def rotation(eta):
    pts=[]
    #reassign pts
    for i in range(x_cell):
        for j in range(y_cell):
            for k in range(deg_cell):
                if oc_grid[i][j][k]!=0:
                    pts.append([i,j,k])
    print(len(pts))

    #for each point - target computed
    for h in pts:
        #compute tgt#
        tgt = [h[0],h[1],(h[2]+eta)]
        prior = oc_grid[h[0]][h[1]][h[2]]
        for i in range(x_cell):
            for j in range(y_cell):
                for k in range(deg_cell):
                    t = normpdf((i*x_cell)+(x_cell//2),tgt[0],x_cell//2)*normpdf((j*y_cell)+(y_cell//2),tgt[1],y_cell//2)*normpdf((k*deg_cell)+(deg_cell//2),tgt[2],deg_cell//2)
                    oc_grid[i][j][k] = (t*prior) + oc_grid[i][j][k]
        #print(h)
        
#translation
def translation(r,eta):
    pts=[]
    #reassign pts
    for i in range(x_cell):
        for j in range(y_cell):
            for k in range(deg_cell):
                if oc_grid[i][j][k]!=0:
                    pts.append([i,j,k])                
    print(len(pts))
    
    #for each point - target computed

    for h in pts:
        #compute tgt#
        tgt = [h[0]+r*math.cos(math.radians(eta)),h[1]+r*math.sin(math.radians(eta)),h[2]]
        prior = oc_grid[h[0]][h[1]][h[2]]
        for i in range(x_cell):
            for j in range(y_cell):
                for k in range(deg_cell):
                    t = normpdf((i*x_cell)+(x_cell//2),tgt[0],x_cell//2)*normpdf((j*y_cell)+(y_cell//2),tgt[1],y_cell//2)*normpdf((k*deg_cell)+(deg_cell//2),tgt[2],deg_cell//2)
                    oc_grid[i][j][k] = (t*prior) + oc_grid[i][j][k]
        #print(h)

def Motion():
    eta=1
    r=1
    print('rotation1')
    rotation(eta)
    print('translation')
    translation(r,eta)
    print('rotation')
    rotation(eta)


Motion()
#to display the 3D matrix.
'''
for i in range(x_cell):
    for j in range(y_cell):
        for k in range(deg_cell):
            #if oc_grid[i][j][k]!=0:
                #pts.append([i,j,k])
            print(oc_grid[i][j][k],end=' ')
        print()
    print()
'''
