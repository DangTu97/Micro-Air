import numpy as np
import socket
import matplotlib.pyplot as plt
from ast import literal_eval
    
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
    else:
        pass
    # rotated 
    drawed_square = np.rot90(drawed_square,rotation)
    return drawed_square


if __name__ == "__main__":
    UDP_IP = "127.0.0.1"
    UDP_PORT = 9877

    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    print("Starting UDP listener at:", UDP_IP, ' port: ', UDP_PORT, sock)

    
    data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    mapping = literal_eval(data.decode())
    #print(mapping)
    
    n = 4
    k = 16
    img = np.zeros((k*n, k*n))
    for i in range(n):
        for j in range(n):
            index = i*n + j
            value = mapping[index]
            #print(value)
            type_id, rotation = value[0], value[1]
            img[i*k:i*k+k,j*k:j*k+k] = draw_map(img[i*k:i*k+k,j*k:j*k+k], type_id, rotation)
    plt.imshow(img)


