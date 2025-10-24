# import mmap
from multiprocessing import shared_memory
import cv2 as cv
import numpy as np
import time
import flatbuffers
import socket
from struct import *

import Multiview.Frame.FrameInfo

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

DATA_PATH = "/home/rajrup/kinect/data/VolumetricVideo/KinectStream/kinect_captures/"
SHM_NAME = "test_shm"

def destroy_shm(shmem):
    # shmem.close()
    shmem.unlink()

def add_shm(shm_name, shm_size):
    flag_shmem = False
    print("Waiting for shared memory ...")
    
    # Wait till the shared memory is created by the server
    while(not flag_shmem):
        try:
            shmem = shared_memory.SharedMemory(name=shm_name, create=False, size=shm_size)
            flag_shmem = True
        except FileNotFoundError:
            time.sleep(1.0/30.0)    # 30 fps
    return shmem

def color_to_opencv(img_buf, img_height, img_width, withAlpha):
    cv_image_with_alpha = np.frombuffer(img_buf, dtype=np.uint8).reshape((img_height, img_width, 4))
    if not withAlpha:
        cv_image_no_alpha = cv.cvtColor(cv_image_with_alpha, cv.COLOR_BGRA2BGR)
        return cv_image_no_alpha
    return cv_image_with_alpha

def depth_to_opencv(img_buf, img_height, img_width):
    return np.frombuffer(img_buf, dtype=np.uint16).reshape((img_height, img_width))

if __name__ == '__main__':
    
    # Example on creating flatbuffers in python
    builder = flatbuffers.Builder(1024)
    Multiview.Frame.FrameInfo.Start(builder)
    Multiview.Frame.FrameInfo.AddFrameNumber(builder, 10)
    Multiview.Frame.FrameInfo.AddViewNumber(builder, 2)
    metadata = Multiview.Frame.FrameInfo.End(builder)
    builder.Finish(metadata)
    buf = builder.Output() # Serialized `bytearray`
    
    
        
    # ---------------------------SETUP----------------------------------
    # Open the file for reading
    date = "Feb_3_2022"
    path = DATA_PATH + date + "/views/"
    frame_number = 1
    view_number = 0
    
    # Read a color image for dimensions. Change it to depth image if you want to share depth image
    img_path = path + f"{frame_number}/color_{view_number}.png"
    imgMat = cv.imread(img_path, cv.IMREAD_UNCHANGED)
    imgSize = imgMat.shape[0] * imgMat.shape[1] * imgMat.shape[2]
    imgShape = imgMat.shape
    print("imgMat shape: ", imgMat.shape)
    print("imgSize: ", imgSize)
    
    print("Creating Socket ...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    
    try:
        # Add shared memory. Waits till the shared memory is created
        shmem = add_shm(SHM_NAME, imgSize)
        
        # ---------------------------RECV----------------------------------
        with conn:
            print(f"Connected by {addr}")
            try:
                while True:
                    data = conn.recv(1024) # Size of integer = 4 bytes
                    if not data:
                        print("No data received, closing connection")
                        break
                    frame_info = Multiview.Frame.FrameInfo.FrameInfo.GetRootAs(data, 0) # Deserialize the data
                    print(f"Received Metadata - Frame Number: {frame_info.FrameNumber()}, View Number: {frame_info.ViewNumber()}")
                    
                    # Read safely from the shared memory
                    shm_buf = shmem.buf
                    img = color_to_opencv(shm_buf, imgShape[0], imgShape[1], True)
                    print("Received Image shape: ", img.shape)
                    
                    out_img_path = path + f"{frame_number}/mmap_color_{view_number}.png"
                    cv.imwrite(out_img_path, img)
                    print(f"Saved Image to {out_img_path}")
                    
            except KeyboardInterrupt:
                pass
    except FileNotFoundError:
        print("Shared memory not found or deleted by the server")
        pass        
    # -------------------------------CLEANUP-----------------------------
    # To Do: There is some problem with the cleanup. It is not working.
    print("Closing connection")
    s.shutdown(socket.SHUT_RDWR)
    s.close()
    destroy_shm(shmem)