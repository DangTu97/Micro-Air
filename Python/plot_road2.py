# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 10:15:26 2020

@author: dang tu
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 10:39:58 2020

@author: dang tu
"""

import numpy as np
import matplotlib.pyplot as plt

def draw_map(square, type_id, rotation=0):
    drawed_square = square
    if type_id == 1: # do nothing
        pass
    
    elif type_id == 2:
        drawed_square[8,0:9] = 1     
        drawed_square[0:9,8] = 1
        
    elif type_id == 3:
        drawed_square[8] = 1
        
    elif type_id == 4:
        drawed_square[8,:] = 1
        drawed_square[:,8] = 1
        drawed_square[8,0:8] = 0
    elif type_id == 5:
        drawed_square[8,:] = 1
        drawed_square[:,8] = 1
    # rotated 
    drawed_square = np.rot90(drawed_square,rotation)
    return drawed_square

'''
road_mapping  = [[1,0], [1,0], [1,0], [3,1],
                 [3,0], [2,1], [1,0], [3,1],
                 [1,0], [2,3], [3,0], [5,0],
                 [1,0], [1,0], [1,0], [3,1]]
'''
road_mapping = [[3,0], [2,1], [3,1], [1,0],
                [1,0], [2,3], [5,0], [3,0],
                [3,0], [3,0], [4,2], [1,0],
                [1,0], [1,0], [3,1], [1,0]]
'''
road_mapping  = [[1,0], [3,1], [1,0], [3,1],
                 [3,0], [4,1], [2,1], [3,1],
                 [1,0], [1,0], [2,3], [5,0],
                 [1,0], [1,0], [1,0], [3,1]]
'''
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
k = 17
img = np.zeros((k*n, k*n))
for i in range(n):
    for j in range(n):
        index = i*n + j
        value = road_mapping[index]
        type_id, rotation = value[0], value[1]
        img[i*k:i*k+k,j*k:j*k+k] = draw_map(img[i*k:i*k+k,j*k:j*k+k], type_id, rotation)
plt.imshow(img)

import pandas as pd 
pd.DataFrame(img).to_csv("test.csv", header=None, index=None)
