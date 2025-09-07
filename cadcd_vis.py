import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 
import struct 
import json
import math



# Define directory containing point cloud files
dir_path = "/media/parvez/Expansion1/cadcd/2019_02_27/0003/labeled/lidar_points/data" 
annotations_path = "/media/parvez/Expansion1/cadcd/2019_02_27/0003/3d_ann.json"



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
        
    def add_geometry(self, geom):
        self.__visualizer.add_geometry(geom)

    def remove_geometry(self, geom):
        self.__visualizer.remove_geometry(geom, reset_bounding_box=False)    
        



        
def create_3d_bbox(center, size, yaw, color=[1, 0, 0]):
    """
    Create a 3D bounding box LineSet for visualization in Open3D.
    center: [x, y, z]
    size: [dx, dy, dz]
    yaw: rotation around z-axis (radians)
    color: RGB for lines
    """
    # CADC -> Open3D mapping: [length, width, height]
    extent = np.array([size[1], size[0], size[2]])   # [l, w, h]
    
    # Create rotation matrix from yaw (rotation about z-axis)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                  [math.sin(yaw),  math.cos(yaw), 0],
                  [0,              0,             1]])
    
    # Create oriented bounding box
    obb = o3d.geometry.OrientedBoundingBox(center, R,  extent)
    
    # Convert OBB to LineSet (so we can color it)
    lineset = o3d.geometry.LineSet.create_from_oriented_bounding_box(obb)
    lineset.paint_uniform_color(color)
    
    return lineset


            
            
            
obj = NonBlockVisualizer()



with open(annotations_path) as f:
   annotations_data = json.load(f)
   

#print(annotations_data)
prev_bboxes = []  # keep track of bboxes to remove them later

i = 0 
for filename in sorted(os.listdir(dir_path)):
    size_float = 4
    list_pcd = []
    if filename.endswith(".bin"):
        # Load point cloud file
        file_path = os.path.join(dir_path, filename)
        with open(file_path, "rb") as f:
           byte = f.read(size_float * 4)
           while byte:
               x, y, z, intensity = struct.unpack("ffff", byte)
               list_pcd.append([x, y, z])
               byte = f.read(size_float * 4)
        
         
        points = np.asarray(list_pcd)
        
        obj.view_control.set_zoom(0.1)
  
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points) 
        
        obj.update_renderer(pcd) 
        
        # === Remove old bounding boxes ===
        for old_bbox in prev_bboxes:
           obj.remove_geometry(old_bbox)
        prev_bboxes = []

        bb_list = [] 
        cuboid = annotations_data[i]["cuboids"]
        for k in range(len(cuboid)):
            if cuboid[k]['position']['x']> 0 and cuboid[k]['position']['x'] < 75: #make sure the cuboid is within the range we want to see
               if cuboid[k]['position']['y'] > -15 and cuboid[k]['position']['y'] < 15:
                  category = cuboid[k]['label'] 
                  center = [cuboid[k]['position']["x"], cuboid[k]["position"]["y"], cuboid[k]["position"]["z"]]
                  size = [cuboid[k]["dimensions"]["x"], cuboid[k]["dimensions"]["y"], cuboid[k]["dimensions"]["z"]]  # [w, l, h]
                  rotation = cuboid[k]["yaw"]   
          
                  # create and add bounding box
                  bbox = create_3d_bbox(center, size, rotation, color=[1, 0, 0])
                  obj.add_geometry(bbox)
                  prev_bboxes.append(bbox)
         
         
        i = i + 1
        time.sleep(0.05)
        

