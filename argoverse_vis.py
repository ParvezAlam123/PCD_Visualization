import numpy as np
import open3d as o3d 
import os 
import pyarrow.feather as feather 



path = "/media/parvez_alam/Expansion/Argoverse 2/Sensor Dataset/Train/train-000/sensor/train"


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


for frame in sorted(os.listdir(path)):
   folder = os.path.join(path,frame, "sensors/lidar")
   
   for frame in os.listdir(folder):
     file_path = os.path.join(folder, frame)
     with open(file_path, 'rb') as f:
       read_df = feather.read_feather(f) 
       points = read_df[['x', 'y', 'z']].to_numpy() 
       
      
       
       
      
       pcd = o3d.geometry.PointCloud()
       pcd.points = o3d.utility.Vector3dVector(points)
      
        
       obj.update_renderer(pcd)








