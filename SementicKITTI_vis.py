import os 
import numpy as np 
import torch 
import torch.nn as  nn 
import struct 
import open3d as o3d 
import time







velo_path = "/media/parvez_alam/Expansion/SementicKitti/data_odometry_velodyne/dataset/sequences/00/velodyne"
label_path = "/media/parvez_alam/Expansion/SementicKitti/data_odometry_labels/dataset/sequences/00/labels"

class NonBlockVisualizer:
    def __init__(self, point_size=1, background_color=[0, 0, 0]):
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









color_dict = {'0':[0, 0, 50], '1': [0, 0, 100], '10': [0, 0, 150], '11':[0, 0, 200], '13':[0,0,250], '15':[0, 25, 50], '16':[0, 50, 50],
             '18':[0, 75, 50], '20':[0, 100, 50], '30':[70, 125, 50], '31':[0, 150, 50], '32':[0, 175, 50], '40':[50, 200, 50], '44':[150, 225, 50], '48':[0, 250, 50], '49':[0, 25, 100], '50':[0, 50, 100], '51':[0, 75, 100], '52':[0, 100, 100], '60':[0, 125, 100], '70':[0, 150, 150], '71':[0, 175, 100], '72':[0, 200, 150], '80':[0, 225, 150], '81':[0, 250, 150], '99':[25, 0, 100], '252':[50, 100, 100], '253':[75, 0, 100], '254':[100, 100, 100], '255':[125, 0, 100], '256':[150, 100, 100], '257':[175, 0, 100], '258':[200, 0, 100], '259':[225, 0, 100]}

velo_order = sorted(os.listdir(velo_path)) 
label_order = sorted(os.listdir(label_path))  


for i in range(len(velo_order)):
    velo_frame = os.path.join(velo_path, velo_order[i])
    label_frame = os.path.join(label_path, label_order[i]) 
    
    size_float = 4
    list_pcd = []
    with open(velo_frame, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
        
         
    points = np.asarray(list_pcd)
    
    label = np.fromfile(label_frame, dtype=np.uint32)
    #label = label.reshape((-1))    
    
    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16    # instance id in upper half  
     
     
     
     
    N, _ = points.shape 
    colors = np.zeros((N, 3)) 
    for col in range(N):
       lab = sem_label[col] 
       c = color_dict[str(lab)] 
       colors[col] = c 
       
    
     
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points) 
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors)) 
    
        
    obj.update_renderer(pcd)
    time.sleep(0.05) 
        



        






