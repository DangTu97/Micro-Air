# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 13:31:59 2020

@author: dang tu
"""
import socket
import time
import numpy as np
import matplotlib.pyplot as plt

def send_json_to_UDP(scan_results):
    # defining the udp endpoint
    UDP_IP = "127.0.0.1"
    UDP_PORT = 9877
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(str(scan_results).encode('utf-8'), (UDP_IP, UDP_PORT))
        print(scan_results)
    except Exception as e:
        print(e)

def draw_map(square, type_id, rotation=0):
    drawed_square = square
    if type_id == 1: # do nothing
        pass
    
    elif type_id == 2:
        drawed_square[7,0:7] = 1
        drawed_square[8,0:7] = 1
        drawed_square[0:9,7] = 1
        drawed_square[0:9,8] = 1 
        
    elif type_id == 3:
        drawed_square[7] = 1
        drawed_square[8] = 1
        
    elif type_id == 4:
        drawed_square[[7,8],:] = 1
        drawed_square[:,[7,8]] = 1
        drawed_square[[7,8],0:7] = 0
    elif type_id == 5:
        drawed_square[[7,8],:] = 1
        drawed_square[:,[7,8]] = 1
    # rotated 
    drawed_square = np.rot90(drawed_square,rotation)
    return drawed_square

road_mapping  = [[1,0], [3,1], [1,0], [3,1],
                 [3,0], [4,1], [2,1], [3,1],
                 [1,0], [1,0], [2,3], [5,0],
                 [1,0], [1,0], [1,0], [3,1]]
'''
road_mapping = [[1,0], [1,0], [3,1], [1,0], [1,0], [3,1], [1,0], [1,0],
                [3,0], [3,0], [4,2], [1,0], [1,0], [3,1], [1,0], [1,0],
                [1,0], [1,0], [3,1], [1,0], [1,0], [3,1], [1,0], [1,0],
                [1,0], [1,0], [2,3], [3,0], [3,0], [5,0], [3,0], [3,0],
                [1,0], [1,0], [1,0], [1,0], [1,0], [3,1], [1,0], [1,0],
                [1,0], [1,0] ,[1,0], [1,0], [1,0], [3,1], [1,0], [1,0],
                [3,0], [3,0], [3,0], [3,0], [3,0], [4,2], [1,0], [1,0],
                [1,0], [1,0], [1,0], [1,0], [1,0], [3,1], [1,0], [1,0]]
'''
n = 4
k = 16
img = np.zeros((k*n, k*n))
for i in range(n):
    for j in range(n):
        index = i*n + j
        value = road_mapping[index]
        type_id, rotation = value[0], value[1]
        img[i*k:i*k+k,j*k:j*k+k] = draw_map(img[i*k:i*k+k,j*k:j*k+k], type_id, rotation)
        
scan_result = np.ndarray.tolist(img.astype(int))
while True:
    time.sleep(2)
    send_json_to_UDP(road_mapping)
    plt.imshow(img)

#import os
#
#while True:
##    time.sleep(1)
#    img = img.T
#    data = img.astype(int)
#    f = 'E://Gama 1.8 Code//intership//includes//img3.csv'
#    if os.path.exists(f):
#        try:
#            os.rename(f, f)
#            print("ok")
#        except OSError as e:
#            print("Error")