import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 
import struct 



# Define directory containing point cloud files
dir_path = "/media/parvez_alam/Expansion/WADS"



class NonBlockVisualizer:
    def __init__(self, point_size=1.5, background_color=[0, 0, 0]):
        self.__visualizer = o3d.visualization.Visualizer()
        self.__visualizer.create_window()
        opt = self.__visualizer.get_render_option()
        opt.background_color = np.asarray(background_color)
        opt = self.__visualizer.get_render_option()
        opt.point_size = point_size

        self.__pcd_vis = o3d.geometry.PointCloud()
        self.__initialized = False
        
        self.view_control = self.__visualizer.get_view_control()
        
       
        
        
        
        
        

    def update_renderer(self, pcd, wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
        else:
            self.__visualizer.update_geometry(self.__pcd_vis)
            
            
        
        #self.view_control.set_up(np.array([0, -1, 1]))
        #self.view_control.set_front(np.array([0, -0.5, -1]))
        #self.view_control.set_lookat([0,0,5])
        #self.view_control.set_zoom(0.1)
        
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()
        
        

        
            
            
            
obj = NonBlockVisualizer()


for sequence in sorted(os.listdir(dir_path)):
    file_dir = os.path.join(dir_path, sequence)
    sequence_number = os.listdir(file_dir)
    pcd_path = os.path.join(file_dir, sequence_number[0], "velodyne")
    for filename in sorted(os.listdir(pcd_path)):
       size_float = 4
       list_pcd = []
       if filename.endswith(".bin"):
           # Load point cloud file
           file_path = os.path.join(pcd_path, filename)
           with open(file_path, "rb") as f:
              byte = f.read(size_float * 4)
              while byte:
                  x, y, z, intensity = struct.unpack("ffff", byte)
                  list_pcd.append([x, y, z])
                  byte = f.read(size_float * 4)
      
           
           points = np.asarray(list_pcd)
        
        
        
  
           pcd = o3d.geometry.PointCloud()
           pcd.points = o3d.utility.Vector3dVector(points)
        
           obj.update_renderer(pcd)
           time.sleep(0.05)
        

        
