import abb1
import numpy as np
import time 
Bin1=[-336,578]
Bin2=[-628,80]
Bin3=[405,-442]
Bin4=[664,14]
Bins=[(Bin1),(Bin2),(Bin3),(Bin4)]

Targets=[(483,252,1),(390,63.5,1),(291,-67,1),(228,-167,1),(147,-317,1),(53,-418,1),(12,-593,1),(0,0,1)]



sin=0.49621 ; cos=-0.8681 

R= abb1.Robot(ip='192.168.125.1')

R.set_speed([600,50,50,50])

time1=time.clock()
for i in range(8):
    
    TargetInfo=Targets[i]
    x=TargetInfo[0]
    y=TargetInfo[1]

    tFollow=0.1         #   sec
    CV=140              #   mm/s
    FollDist=CV*tFollow     #FollDist on Y direction
    
# Convert target's coord into Robots Coord
    #xR=cos*x-sin*y
    #yR=sin*x+cos*y
    xR=x
    yR=y
     #+FollDist

    xRPicked=x           #cos*x-sin*y
    yRPicked=y           #sin*x+cos*y

    Bin=Bin2

    theta=np.arctan2((Bin[1]-yRPicked),(Bin[0]-xRPicked))
    
    a=np.array((xRPicked,yRPicked))
    b=np.array((Bin[0],Bin[1]))
    dist=np.linalg.norm(a-b)
    distMoveL=dist*0.6

    xMoveL=x+np.cos(theta)*distMoveL
    yMoveL=y+np.sin(theta)*distMoveL



    #R.Pick_n_Toss([[xR,yR,-1300], [2,xMoveL,yMoveL,2]])

    R.Pick_n_Toss([[xR,yR,-1380], [0.05,xMoveL,yMoveL,0.15]])

time2=time.clock()-time1
print(time2) 


#TargetInfo=[0,0,1]  #   X, Y , kind 0,1,2,3

 
#R.Yoyo([[xR,yR,-1380], [3,xMoveL,yMoveL,1]])



#R.set_cartesian([[Bin4[0],Bin4[1],-1250], [0,0,1,0]])  

 


