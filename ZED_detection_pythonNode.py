# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 14:03:44 2019

@author: anasa
"""

#importing modules

import cv2
import numpy as np
import socket
import struct
import os
import configparser
import sys
import wget
import matplotlib.pyplot as plt
import time
from keras.models import load_model
from keras.preprocessing import image

def download_calibration_file(serial_number) :
    if os.name == 'nt' :
        hidden_path = 'D:/Downloads/'
        calibration_file = hidden_path + 'SN' + str(serial_number) + '.conf'
    else :
        hidden_path = 'D:/Downloads/'
    calibration_file = hidden_path + 'SN' + str(serial_number) + '.conf'

    if os.path.isfile(calibration_file) == False:
        url = 'http://calib.stereolabs.com/?SN='
        filename = wget.download(url=url+str(serial_number), out=calibration_file)

        if os.path.isfile(calibration_file) == False:
            print('Invalid Calibration File')
            return ""

    return calibration_file

def init_calibration(calibration_file, image_size) :

    cameraMarix_left = cameraMatrix_right = map_left_y = map_left_x = map_right_y = map_right_x = np.array([])

    config = configparser.ConfigParser()
    config.read(calibration_file)

    check_data = True
    resolution_str = ''
    if image_size.width == 2208 :
        resolution_str = '2K'
    elif image_size.width == 1920 :
        resolution_str = 'FHD'
    elif image_size.width == 1280 :
        resolution_str = 'HD'
    elif image_size.width == 672 :
        resolution_str = 'VGA'
    else:
        resolution_str = 'HD'
        check_data = False

    T_ = np.array([-float(config['STEREO']['Baseline'] if 'Baseline' in config['STEREO'] else 0),
                   float(config['STEREO']['TY_'+resolution_str] if 'TY_'+resolution_str in config['STEREO'] else 0),
                   float(config['STEREO']['TZ_'+resolution_str] if 'TZ_'+resolution_str in config['STEREO'] else 0)])


    left_cam_cx = float(config['LEFT_CAM_'+resolution_str]['cx'] if 'cx' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_cy = float(config['LEFT_CAM_'+resolution_str]['cy'] if 'cy' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_fx = float(config['LEFT_CAM_'+resolution_str]['fx'] if 'fx' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_fy = float(config['LEFT_CAM_'+resolution_str]['fy'] if 'fy' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_k1 = float(config['LEFT_CAM_'+resolution_str]['k1'] if 'k1' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_k2 = float(config['LEFT_CAM_'+resolution_str]['k2'] if 'k2' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_p1 = float(config['LEFT_CAM_'+resolution_str]['p1'] if 'p1' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_p2 = float(config['LEFT_CAM_'+resolution_str]['p2'] if 'p2' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_p3 = float(config['LEFT_CAM_'+resolution_str]['p3'] if 'p3' in config['LEFT_CAM_'+resolution_str] else 0)
    left_cam_k3 = float(config['LEFT_CAM_'+resolution_str]['k3'] if 'k3' in config['LEFT_CAM_'+resolution_str] else 0)


    right_cam_cx = float(config['RIGHT_CAM_'+resolution_str]['cx'] if 'cx' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_cy = float(config['RIGHT_CAM_'+resolution_str]['cy'] if 'cy' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_fx = float(config['RIGHT_CAM_'+resolution_str]['fx'] if 'fx' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_fy = float(config['RIGHT_CAM_'+resolution_str]['fy'] if 'fy' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_k1 = float(config['RIGHT_CAM_'+resolution_str]['k1'] if 'k1' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_k2 = float(config['RIGHT_CAM_'+resolution_str]['k2'] if 'k2' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_p1 = float(config['RIGHT_CAM_'+resolution_str]['p1'] if 'p1' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_p2 = float(config['RIGHT_CAM_'+resolution_str]['p2'] if 'p2' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_p3 = float(config['RIGHT_CAM_'+resolution_str]['p3'] if 'p3' in config['RIGHT_CAM_'+resolution_str] else 0)
    right_cam_k3 = float(config['RIGHT_CAM_'+resolution_str]['k3'] if 'k3' in config['RIGHT_CAM_'+resolution_str] else 0)

    R_zed = np.array([float(config['STEREO']['RX_'+resolution_str] if 'RX_' + resolution_str in config['STEREO'] else 0),
                      float(config['STEREO']['CV_'+resolution_str] if 'CV_' + resolution_str in config['STEREO'] else 0),
                      float(config['STEREO']['RZ_'+resolution_str] if 'RZ_' + resolution_str in config['STEREO'] else 0)])

    R, _ = cv2.Rodrigues(R_zed)
    cameraMatrix_left = np.array([[left_cam_fx, 0, left_cam_cx],
                         [0, left_cam_fy, left_cam_cy],
                         [0, 0, 1]])

    cameraMatrix_right = np.array([[right_cam_fx, 0, right_cam_cx],
                          [0, right_cam_fy, right_cam_cy],
                          [0, 0, 1]])

    distCoeffs_left = np.array([[left_cam_k1], [left_cam_k2], [left_cam_p1], [left_cam_p2], [left_cam_k3]])

    distCoeffs_right = np.array([[right_cam_k1], [right_cam_k2], [right_cam_p1], [right_cam_p2], [right_cam_k3]])

    T = np.array([[T_[0]], [T_[1]], [T_[2]]])
    R1 = R2 = P1 = P2 = np.array([])

    R1, R2, P1, P2 = cv2.stereoRectify(cameraMatrix1=cameraMatrix_left,
                                       cameraMatrix2=cameraMatrix_right,
                                       distCoeffs1=distCoeffs_left,
                                       distCoeffs2=distCoeffs_right,
                                       R=R, T=T,
                                       flags=cv2.CALIB_ZERO_DISPARITY,
                                       alpha=0,
                                       imageSize=(image_size.width, image_size.height),
                                       newImageSize=(image_size.width, image_size.height))[0:4]

    map_left_x, map_left_y = cv2.initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, (image_size.width, image_size.height), cv2.CV_32FC1)
    map_right_x, map_right_y = cv2.initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, (image_size.width, image_size.height), cv2.CV_32FC1)

    cameraMatrix_left = P1
    cameraMatrix_right = P2

    return cameraMatrix_left, cameraMatrix_right, map_left_x, map_left_y, map_right_x, map_right_y

class Resolution :
    width = 1920
    height =1080





host = socket.gethostname()  # as both code is running on same pc
port = 5000  # socket server port number

client_socket = socket.socket()  # instantiate

def send_data(a,b,c):

    values=(a,b,c)
    packer = struct.Struct('I f f')
    packed_data=packer.pack(*values)
    client_socket.send(packed_data)

client_socket.connect((host, port))  # connect to the server

#print('wait 10 seconds for Conv Speed')
#arduino=serial.Serial('COM4',9600)
#cvRead=np.zeros((10))
#i=0
#while(i<10):
#    myData= (arduino.readline().strip())
#    cvRead[i]=float(myData.decode('utf-8'))
#    i=i+1
#arduino.close()
#cv=sum(cvRead) / len(cvRead)
#print('Conveyor Speed=',cv,'mm/s')
cap = cv2.VideoCapture(0)
if cap.isOpened() == 0:
    exit(-1)

image_size = Resolution()
image_size.width = 1920
image_size.height = 1080

# Set the video resolution to HD720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width*2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

serial_number = 12970 # int(sys.argv[1])

calibration_file = download_calibration_file(serial_number)
if calibration_file  == "":
    exit(1)
print("Calibration file found. Loading...")

camera_matrix_left, camera_matrix_right, map_left_x, map_left_y, map_right_x, map_right_y = init_calibration(calibration_file, image_size)
red_lower=np.array([0,0,100],np.uint8)
red_upper=np.array([250,255,255],np.uint8)
t0=time.clock()
cv=135
LastDetectionTime=t0
LastLowestBlob=200
LastLowestBlobDetectionTime=time.clock()
BlobDetectionFrequency=2
i=0
blobsINFO=np.zeros((2000,3))
print('yasuMaria')
k=0
    
loaded_model =load_model('test_model_3_best150_100_classes.h5')
loaded_model.layers[0].input_shape #(None, 160, 160, 3)

    
while True :
    t=time.clock()-t0
    #print(t-LastDetectionTime,1/BlobDetectionFrequency)
        
    if ((abs(t-LastDetectionTime))>1/BlobDetectionFrequency):
        LastDetectionTime=t
        print('click')    
        # Get a new frame from camera
        retval, frame = cap.read()
        # Extract left and right images from side-by-side
        left_right_image = np.split(frame, 2, axis=1)
        # Display images
        #right_image=left_right_image[0:1080,1920:3000]
        #left_rect = cv2.remap(left_right_image[0], map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)
        right_rect = cv2.remap(left_right_image[1], map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)
        
        
        img=right_rect
        img=img[0:1080,375:1815]
        cv2.imshow("left RECT", img)
        if cv2.waitKey(30) >= 0 :
            break
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV) 
        red=cv2.inRange(hsv, red_lower, red_upper)

        kernal = np.ones((5 ,5), "uint8")        
        red=cv2.dilate(red,kernal)
        backgr = cv2.imread('C:/Users/fredy/Documents/ptyxiakh/python scripts/ZED_detection/backround.png')
        
        
            
        lowestYdetection=50
        _, contours, _=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>15000) and (area<500000):
                x,y,w,h = cv2.boundingRect(contour)
                if ((y+h/2)>400) and ((y+h/2<680)):
                        #img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                        
                    absPosX=int((0.5381*(1440-(x+w/2)-720)))        # (-1.2422)
                    absPosY=int((-0.574*(1080-(y+h/2)-540))-1424)
                    print('target in')
                    if (((absPosY<(LastLowestBlob+cv*(LastDetectionTime-LastLowestBlobDetectionTime)-100)) ) or ((absPosY>(LastLowestBlob+cv*(LastDetectionTime-LastLowestBlobDetectionTime)+100)))) :

                        TrashIMG=backgr    
                        h = int(h) if ((int(h) % 2) == 0) else int(h) - 1
                        w = int(w) if ((int(w) % 2) == 0) else int(w) - 1
                        offset=0 
                        Trash= np.zeros((h+(offset*2),w+(offset*2),3))
                        Trash=img[(y-offset):(y+offset+h),(x-offset):(x+offset+w)]  
                        a=(399-(h/2)-offset)
                        b=(399+(h/2)+offset)
                            
                        c=(399-(w/2)-offset)
                        d=(399+(w/2)+offset)
                                                     
                        if (Trash.shape[2]==TrashIMG.shape[2]) and (Trash.shape[0]<TrashIMG.shape[0]) and  (Trash.shape[1]<TrashIMG.shape[1]) :
                            print('OK')  
                            TrashIMG[int(a):int(b),int(c):int(d)] = Trash
                            TrashIMGD = cv2.cvtColor(TrashIMG, cv2.COLOR_RGB2BGR) # plt allo apo cv2
                            dim=(150,150)
                            img = cv2.resize(TrashIMGD,dim,interpolation= cv2.INTER_AREA)
                        
                            img = np.expand_dims(img, axis=0)
                            result=loaded_model.predict_classes(img)
                            if (result==0):
                                print('ALUMINUM',absPosX,absPosY)
                                cv2.putText(TrashIMG,"ALUMINUM",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),3)
                            elif (result==1):
                                print('CARTON',absPosX,absPosY)
                                cv2.putText(TrashIMG,"CARTON",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0),3)
                            elif (result==2):
                                print('PET',absPosX,absPosY)
                                cv2.putText(TrashIMG,"PET",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0),3)
                            blobsINFO[i,0]=result
                            blobsINFO[i,1]=absPosX
                            blobsINFO[i,2]=absPosY    
                            send_data(int(result),absPosX,absPosY)
                            

                            cv2.imshow("left RECT", TrashIMG)
                            if cv2.waitKey(30) >= 0 :
                                #plt.imsave('name.png', array)
                                break
                            i=i+1
                    else:     
                        print("ignored")
                          
                    if (absPosY<lowestYdetection):
                        lowestYdetection=absPosY
                    
                    

        LastLowestBlob=lowestYdetection
        LastLowestBlobDetectionTime=LastDetectionTime  
client_socket.close()










        
        
        
        
    



