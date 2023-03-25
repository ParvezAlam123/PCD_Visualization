import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 


# Define directory containing point cloud files
dir_path = "/home/parvez/Data/Dataset/training/SteeringData/"



class NonBlockVisualizer:
    def __init__(self, point_size=2, background_color=[0, 0, 0]):
        self.__visualizer = o3d.visualization.Visualizer()
        self.__visualizer.create_window(window_name='Open3D', width=800, height=600)
        opt = self.__visualizer.get_render_option()
        opt.background_color = np.asarray(background_color)
        opt = self.__visualizer.get_render_option()
        opt.point_size = point_size

        self.__pcd_vis = o3d.geometry.PointCloud()
        self.__initialized = False
        self.__lineset_list = []
        
        # Enable mouse controls
        self.__visualizer.run()

    def update_renderer(self, pcd,linesets=None, wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
        else:
            self.__visualizer.update_geometry(self.__pcd_vis)
            
            
        # Remove previous linesets
        for lineset in self.__lineset_list:
            self.__visualizer.remove_geometry(lineset)

        if linesets is not None:
            for lineset in linesets:
                if lineset is not None:
                    self.__visualizer.add_geometry(lineset)
        
        self.__lineset_list = linesets
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()

        if wait_time > 0:
            time.sleep(wait_time)
            
            
            
obj = NonBlockVisualizer()


for filename in sorted(os.listdir(dir_path)):
    if filename.endswith(".pcd"):
    
        # Load point cloud file
        pcd = o3d.io.read_point_cloud(os.path.join(dir_path, filename))
        
        # Create linesets for each object in the point cloud
        linesets = []
        for object in objects:
            lineset = create_lineset(object.bounding_box)
            linesets.append(lineset)
               
            
        obj.update_renderer(pcd, linesets)
        
        
        
        
        
        
        
        
        
        
        
        
        
