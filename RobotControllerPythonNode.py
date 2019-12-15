import abb1
import numpy as np
import serial
import socket
from threading import Thread
import time
import struct

#Camera data Grabber Init
host = socket.gethostname()
port = 5000  # initiate port no above 1024
server_socket = socket.socket()  # get instance

#Controller Init
TargetsList=np.zeros((2000, 4))
ID=0
Target=False
Tclock=0
#Conveyor belt encoder Init
encoder=0 ; MMperCount=0.741806782 ; ConvSpeed=0  ; encoder1=0 ; t1=0; SpeedTStamp=0; SpeedFrequency=2  #Hz ConvSpeed Calc freq

Sorted=0 ; Missed=0 
PetBin=[-320,566]
AluBin=[-600,76]
CartonBin=[430,-458]
Bin=[0,0]
#Robot Initilisation
CartesianSpeed=5000 #mm/s

R = abb1.Robot(ip='192.168.125.1')
R.set_speed([CartesianSpeed,50,50,50]) 
R.set_zone('z200')               #set speed
R.set_cartesian([[0,0,-1200], [0,0,1,0]])  # set to zero

sin=0.4962; cos=-0.8681
Panic=False
Vacuum=False
Placed=False

def thread1(threadname):
    global  TargetsList,Tclock, Target, Bin, PrevBin, encoder,t1,ConvSpeed,encoderTStamp,SpeedTStamp, Panic,Vacuum,Sorted,Missed # Optional if you treat a as read-only
    while (1):
        Target=False
        #print(Tclock)
        print(encoder)
        print(ID)
        print('ConvSpeed',ConvSpeed,'mm/s')
        #print('Vacuum',Vacuum)        
        #print('Panic',Panic)
        #print('Sorted items',Sorted)
        #print('Missed items',Missed)
        for i in range(0, ID):
            #print(encoder,TargetsList[i,1],encoder-TargetsList[i,1])
            
           
            if encoder-TargetsList[i,1]>-400 and encoder-TargetsList[i,1]<400 and TargetsList[i,3]==0: 

                          
                TargetData=[TargetsList[i,0],(encoder-TargetsList[i,1]),TargetsList[i,2],TargetsList[i,1]]
                print(TargetData)
                
                
                print('robot grab it---------------------------------------------------------------------------------------------------------------------')
                #Bin,Placed=Pick_n_Place(TargetData,ConvSpeed,Bin)
                Bin,Placed=Pick_n_Toss(TargetData,ConvSpeed,Bin)
                Placed=True
                if Placed==True :
                    Sorted=Sorted+1
                    TargetsList[i,3]=1  # Target Selected
                else:
                    Missed=Missed+1
                    TargetsList[i,3]=1
                break
#        if Target==True:
#            print('robot grab it---------------------------------------------------------------------------------------------------------------------')
#            print('robot grab it---------------------------------------------------------------------------------------------------------------------')
#
#            time.sleep(3)
#            Sorted=Sorted+1
#            #Bin,Sorted,Missed=Pick_n_Place(TargetData,ConvSpeed,Bin,Sorted,Missed)          
#        
#                
                
def thread2(threadname):
    global ID, TargetList,t0,encoder,Tclock,ConvSpeed
    ID=0
    server_socket.bind((host, port))  # bind host address and port together
    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))
    t0=time.clock()
    unpacker= struct.Struct('I f f f')
    delay=0
    while True: 
        data = conn.recv(unpacker.size)                       #take data
        unpacked_data = unpacker.unpack(data)                 #unpack
        if not data:
            break
        Class,X,Y,delay = unpacked_data                                
        
        #Vacuum,encoder,encoderTStamp=readArduino()
        TargetsList[ID,:]=[X,(float(encoder)-Y)-(delay*ConvSpeed),Class,False]   #Save data at Targetlist for robot controller
        print('New Target',TargetsList[ID,:])
        Tclock=time.clock()
        ID=ID+1
    conn.close()  # close the connection

def thread3(threadname):
    global encoder,ConvSpeed,Vacuum,Panic
    s=serial.Serial('COM5',9600)
    encoderTStamp=time.clock()
    SpeedFrequency=10
    SpeedTStamp=time.clock()
    encoderP=0
    a=1/6
    while(1):
        myData= s.readline()
        Vacuum=myData.decode('utf-8', errors='ignore')
        if int(Vacuum)>350:
            Vacuum=True
        else: Vacuum=False
        myData= s.readline()
        encoder=int(myData.decode('utf-8', errors='ignore'))*0.8167641   #0.78113 #0.741806782
        encoderTStamp=time.clock()
        if ((float(encoderTStamp)-float(SpeedTStamp))>1/SpeedFrequency):
            ConvSpeedRaw=(encoder-encoderP)/(encoderTStamp-SpeedTStamp)
            ConvSpeed=(1-a)*ConvSpeed+a*ConvSpeedRaw   # 1st order loaw pass filter
            encoderP=encoder
            SpeedTStamp=encoderTStamp
            if ConvSpeed<100: Panic=True; print('--------------Conveyor is Stopped-----------------')
            else: Panic=False
    
         

def Pick_n_Place(TargetData,ConvSpeed,Bin):
    global Vacuum,encoder,Panic
    
    x=TargetData[0]
    y=TargetData[1]
    a = np.array((Bin[0] ,Bin[1]))
    b = np.array((x,y))
    dist=np.linalg.norm(a-b)
    #offset=0.02*(dist-300)
    #PickingTime=0.2+dist/CartesianSpeed
    xR=(-0.8681)*x-0.4962*y
    yR=0.4962*x-0.8681*y
    Placed=False
    if abs(xR)<600 and abs(yR)<600: 
        if TargetData[2]==0:
            zoff=-1332 + 10
            Bin=AluBin
        elif TargetData[2]==1:
            zoff=-1376  + 10
            Bin=CartonBin
        elif TargetData[2]==2:
            zoff=-1380  + 10
            Bin=PetBin
        y=((int(encoder)-TargetData[3]))+(0.2+0.1)*ConvSpeed+90
        xR=(-0.8681)*x-0.4962*y
        yR=0.4962*x-0.8681*y
        
        R.set_cartesianBLUE([[xR,yR,zoff], [0.2,0.1,ConvSpeed,0.3]])
        time.sleep(0.2)
        
        
        if Vacuum==True :
            R.set_cartesianTime([[Bin[0],Bin[1],-1250], [0,0,1,0.2]])
            
            print('Target Sorted',TargetData)
            Placed=True
        else:
            R.set_cartesianTime([[Bin[0],Bin[1],-1250], [0,0,1,0.2]])
            print('Target missed',TargetData)
            Bin=[xR,yR]
            Placed=False
    return Bin,Placed

def Pick_n_Toss(TargetData,ConvSpeed,Bin):
    global Vacuum,encoder,Panic
    FollDist=ConvSpeed*0.1
    x=TargetData[0]
    y=TargetData[1]+FollDist
    xRPicked=(-0.8681)*x-0.4962*y
    yRPicked=0.4962*x-0.8681*y
    
    if abs(xRPicked)<600 and abs(yRPicked)<600: 
        if TargetData[2]==0:
            zoff=-1332
            Bin=AluBin
        elif TargetData[2]==1:
            zoff=-1376
            Bin=CartonBin
        elif TargetData[2]==2:
            zoff=-1380+5
            Bin=PetBin
        theta=np.arctan2((Bin[1]-yRPicked),(Bin[0]-xRPicked))
        a=np.array(xRPicked,yRPicked)
        b=np.array((Bin[0],Bin[1]))   ## https://stackoverflow.com/questions/50973041/typeerror-data-type-not-understood-numpy-zeros
        dist=np.linalg.norm(a-b)
        
        
        
        
        
        
        distMoveL=0.6*dist
        if dist<400 or TargetData[2]==1:
            distMoveL=0.8*dist
    
        xMoveL=xRPicked+np.cos(theta)*distMoveL
        yMoveL=yRPicked+np.sin(theta)*distMoveL
    
    
        y=((int(encoder)-TargetData[3]))+(0.05+0.1)*ConvSpeed+50 #+90
        xR=(-0.8681)*x-0.4962*y
        yR=0.4962*x-0.8681*y
        
        a = np.array((0,0))
        b = np.array((xMoveL,yMoveL))
        dist=np.linalg.norm(a-b)
        if abs(dist)<750:
            R.Pick_n_Toss([[xR,yR,zoff],[0.05,xMoveL,yMoveL,0.15]])
        else:
            print(xMoveL,yMoveL)
        
    return Bin,Placed


#def readArduino():
#    myData= s.readline()
#    Vacuum=myData.decode('utf-8', errors='ignore')
#    if int(Vacuum)>350:
#        Vacuum=True
#    else: Vacuum=False
#    myData= s.readline()
#    encoder=myData.decode('utf-8', errors='ignore')
#    encoderTStamp=time.clock()
#    return bool(Vacuum),int(encoder)*0.741806782 ,encoderTStamp
#    
#    
#def ComputeConvSpeed(encoder1,t1,encoder2,t2,ConvSpeed):
#    ConvSpeedRaw=(encoder2-encoder1)/(t2-t1)
#    a=1/6
#    ConvSpeed=(1-a)*ConvSpeed+a*ConvSpeedRaw   # 1st order loaw pass filter
#    
#    return ConvSpeed 
#   

#def Pick_n_Spin:
    
#angle2=180-angle
#offset_elips=EMAma[1]*0.25      
#Xpick=np.sin(angle2*0.0174532925)*offset_elips
#Ypick=np.cos(angle2*0.0174532925)*offset_elips
#Xpix=Exy[0]+Xpick
#Ypix=Exy[1]+Ypick
#cv2.putText(img,".Exy",(int(Exy[0]),int(Exy[1])),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255,5))
#cv2.putText(img,".",(int(Xpix),int(Ypix)),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),5)
#            
            
                




















#def Target_Planner:
#def Target_Toss_Planner:
#def GUI:
#def PlotStats:




thread1a = Thread( target=thread1, args=("Thread-1", ) )
thread2a = Thread( target=thread2, args=("Thread-2", ) )
thread3a = Thread( target=thread3, args=("Thread-2", ) )

thread1a.start()
thread2a.start() 
thread3a.start()  
 
thread1a.join()
thread2a.join()
thread3a.join()