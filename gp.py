import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 


# Define directory containing point cloud files
dir_path = "/home/parvez_alam/Dataset/training/SteeringData"



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

    def update_renderer(self, pcd, wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
        else:
            self.__visualizer.update_geometry(self.__pcd_vis)
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()

        if wait_time > 0:
            time.sleep(wait_time)
            
            
            
obj = NonBlockVisualizer()


for filename in sorted(os.listdir(dir_path)):
    if filename.endswith(".pcd"):
    
        # Load point cloud file
        pcd = o3d.io.read_point_cloud(os.path.join(dir_path, filename))
        
        points = np.asarray(pcd.points)
        
        
        # Get the elevation values of all points in the point cloud
        elevation = points[:, 2]
       
        # Set a threshold height value for ground points
        threshold = -0.5
        
        # Identify the indices of ground points based on the threshold height value
        ground_indices = np.where(elevation <= threshold)
        
        # Remove the ground points from the point cloud
        points = np.delete(points, ground_indices, axis=0)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        obj.update_renderer(pcd)
        
        
        
        
        
        
        
        
        
