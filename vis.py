import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 


# Define directory containing point cloud files
dir_path = "/home/parvez_alam/Dataset/validation/SteeringData"



class NonBlockVisualizer:
    def __init__(self, point_size=2, background_color=[0, 0, 0]):
        self.__visualizer = o3d.visualization.Visualizer()
        self.__visualizer.create_window()
        opt = self.__visualizer.get_render_option()
        opt.background_color = np.asarray(background_color)
        opt = self.__visualizer.get_render_option()
        opt.point_size = point_size

        self.__pcd_vis = o3d.geometry.PointCloud()
        self.__initialized = False

    def update_renderer(self, pcd,bbox_lines, wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
            self.__visualizer.add_geometry(bbox_lines)
            
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
        
        # Define the corners of the bounding box
        bbox_corners = np.array([[10, -5, -2], [15, -5, -2], [10, 5, -2], [10, 5, -2],
                         [10, -5, 3], [15, -5, 3], [15, 5, 3], [10, 5, 3]])

        # Define the edges of the bounding box
        bbox_edges = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                       [4, 5], [5, 6], [6, 7], [7, 4],
                       [0, 4], [1, 5], [2, 6], [3, 7]])
                       
        # Define the color of the bounding box
        bbox_color = [1, 0, 0] # Red

        # Create the bounding box line set
        bbox_lines = o3d.geometry.LineSet()
        bbox_lines.points = o3d.utility.Vector3dVector(bbox_corners)
        bbox_lines.lines = o3d.utility.Vector2iVector(bbox_edges)
        bbox_lines.colors = o3d.utility.Vector3dVector([bbox_color for i in range(len(bbox_edges))])
        

        
        obj.update_renderer(pcd, bbox_lines)
        
