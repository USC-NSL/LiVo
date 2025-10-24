import open3d as o3d
import os
import numpy as np
import time


def read_ptcl(dir, frameID):
    filepath = os.path.join(dir, "{}.bytes".format(frameID))
    with open(filepath, mode='rb') as file: # b is important -> binary
        data = file.read()
    file.close()
    return data

def parse_ptcl(data):
    size = len(data)
    num_points = size // 15
    xyz = np.zeros((num_points, 3), dtype=np.float64)
    rgb = np.zeros((num_points, 3), dtype=np.uint8)
    rgb_norm = np.zeros((num_points, 3), dtype=np.float64)
    for i in range(num_points):
        xyz[i] = np.frombuffer(data[i*15:i*15+12], dtype=np.float32)
        rgb_norm[i] = np.frombuffer(data[i*15+12:i*15+15], dtype=np.uint8)/255.0
    return xyz, rgb_norm

if __name__ == "__main__":
    ptcl_dir = "/datassd/KinectStream/panoptic_captures/kinoptic_ptclouds"
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    start = 3000
    end = 3100
    
    points = []
    colors = []
    
    for frameID in range(start, end):
        binary_data = read_ptcl(ptcl_dir, frameID)
        
        start_time = time.time()
        xyz, rgb = parse_ptcl(binary_data)
        points.append(o3d.utility.Vector3dVector(xyz))
        colors.append(o3d.utility.Vector3dVector(rgb))
        elapsed_time = (time.time() - start_time)*1000.0 # in ms
        print("Time to parse to o3d: {}".format(elapsed_time))
    
    pcd = o3d.geometry.PointCloud()
    i = nframes = 0
    start_time = time.time()
    while(True):
        pcd.points = points[i]
        pcd.colors = colors[i]
        if nframes == 0:
            vis.add_geometry(pcd)
        else:
            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        
        nframes += 1
        i = (i+1) % (end - start)
        
        elapsed_time = (time.time() - start_time)*1000.0 # in ms
        fps = nframes / elapsed_time * 1000.0
        print("FPS: {}".format(fps))
        
        
        
    
    
        