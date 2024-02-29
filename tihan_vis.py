import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 
import struct 
import json 



# Define directory containing point cloud files
dir_path = "/media/parvez_alam/Expansion/TiHAN_LiDAR/Scene5/lidar"
label_path = "/media/parvez_alam/Expansion/TiHAN_LiDAR/Scene5/label"


global lines 
lines = [] 
 




class NonBlockVisualizer:
    def __init__(self, point_size=1, background_color=[255,255, 255]):
        self.__visualizer = o3d.visualization.Visualizer()
        self.__visualizer.create_window()
        opt = self.__visualizer.get_render_option()
        opt.background_color = np.asarray(background_color)
        opt = self.__visualizer.get_render_option()
        opt.point_size = point_size

        self.__pcd_vis = o3d.geometry.PointCloud()
        self.__initialized = False
        # Create view control
        self.view_control = self.__visualizer.get_view_control()
       
        
        self.total_bboxes = []
        for i in range(500):
           self.total_bboxes.append(o3d.geometry.LineSet())
        
        self.count_bb = 0
     
         
        
        
        
        
    
    def add_bounding_box(self, center, length, width, height, orientation):
       # Define the vertices of the bounding box
       vertices = np.asarray([
             [length/2, width/2, height/2],
             [-length/2, width/2, height/2],
             [-length/2, -width/2, height/2],
             [length/2, -width/2, height/2],
             [length/2, width/2, -height/2],
             [-length/2, width/2, -height/2],
             [-length/2, -width/2, -height/2],
             [length/2, -width/2, -height/2]])

       # Rotate the vertices based on the orientation
       R = np.array([[np.cos(orientation), -np.sin(orientation), 0],
                  [np.sin(orientation), np.cos(orientation), 0],
                  [0, 0, 1]])
       vertices = vertices @ R.T

       # Translate the vertices to the center of the bounding box
       vertices = vertices + center

       # Define the edges of the bounding box
       edges = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]])

       # Create the LineSet geometry
       #line_set = o3d.geometry.LineSet()
       #line_set.points = o3d.utility.Vector3dVector(vertices)
       #line_set.lines = o3d.utility.Vector2iVector(edges)

       # Set the colors of the edges
       #colors = [[1, 0, 0] for i in range(len(edges))]
       #line_set.colors = o3d.utility.Vector3dVector(colors)
       
       
       
 
       
       
       return vertices, edges
       
       
    

    def update_renderer(self, pcd, bb_list, wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors
        
        
        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
            
            for dim_of_bb in bb_list:
               vertices, edges = self.add_bounding_box(dim_of_bb[0], dim_of_bb[1], dim_of_bb[2], dim_of_bb[3], dim_of_bb[4]) 
               obj_type = dim_of_bb[5] 
        
               #self.__visualizer.add_geometry(line_set)
               #lines.append(line_set)
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices)
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               
               if obj_type == 'Trimotorcycle':
                  colors = [[0, 0, 0.2] for i in range(len(edges))] 
                  
               if obj_type == 'Motorcycle':
                  colors = [[0, 0, 0.4] for i in range(len(edges))]
                  
               if obj_type == 'Scooter':
                  colors = [[0, 0, 0.6] for i in range(len(edges))]
                  
               if obj_type == 'Car':
                  colors = [[0, 1, 0.8] for i in range(len(edges))]
                  
               if obj_type == 'Pedestrian':
                  colors = [[0, 0, 1] for i in range(len(edges))] 
                  
               if obj_type == "Bus":
                  colors = [[0, 0.2, 0] for i in range(len(edges))]
                  
               if obj_type == "Unknown1":
                  colors = [[0, 0.4, 0] for i in range(len(edges))]
                  
              
                
               
                  
                  
             
                 
                        
                  
                  
               self.total_bboxes[self.count_bb].colors = o3d.utility.Vector3dVector(colors) 
               self.count_bb = self.count_bb + 1
             
            for bbox in self.total_bboxes:
               self.__visualizer.add_geometry(bbox)
               
            
               
               
                  
        else:
            self.__visualizer.update_geometry(self.__pcd_vis)
            
            for i in range(self.count_bb):
               self.total_bboxes[i].points = o3d.utility.Vector3dVector([])
               self.total_bboxes[i].lines = o3d.utility.Vector2iVector([]) 
               self.total_bboxes[i].colors = o3d.utility.Vector3dVector([])
               
            self.count_bb = 0
            for dim_of_bb in bb_list:
               vertices, edges = self.add_bounding_box(dim_of_bb[0], dim_of_bb[1], dim_of_bb[2], dim_of_bb[3], dim_of_bb[4]) 
               obj_type = dim_of_bb[5] 
               
               
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices)
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               
               if obj_type == 'Trimotorcycle':
                  colors = [[0, 0, 0.2] for i in range(len(edges))] 
                  
               if obj_type == 'Motorcycle':
                  colors = [[0, 0, 0.4] for i in range(len(edges))]
                  
               if obj_type == 'Scooter':
                  colors = [[0, 0, 0.6] for i in range(len(edges))]
                  
               if obj_type == 'Car':
                  colors = [[0, 0, 0.8] for i in range(len(edges))]
                  
               if obj_type == 'Pedestrian':
                  colors = [[0, 0, 1] for i in range(len(edges))] 
                  
               if obj_type == "Bus":
                  colors = [[0, 0.2, 0] for i in range(len(edges))]
                  
               if obj_type == "Unknown1":
                  colors = [[0, 0.4, 0] for i in range(len(edges))]
                  
                  
                  
             
                 
               
               self.total_bboxes[self.count_bb].colors = o3d.utility.Vector3dVector(colors) 
               self.count_bb = self.count_bb + 1
               
               
            for bbox in self.total_bboxes:
               self.__visualizer.update_geometry(bbox)
               
               
               
               
            
            """for i in range(len(lines)):
               self.__visualizer.remove_geometry(lines[i])
               
            lines.clear() 
          
            for dim_of_bb in bb_list:
               line_set = self.add_bounding_box(dim_of_bb[0], dim_of_bb[1], dim_of_bb[2], dim_of_bb[3], dim_of_bb[4], )
               self.__visualizer.add_geometry(line_set)
               lines.append(line_set) """
        
          
        
           
        
        
                 
        
       
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()
        
        
        #self.__visualizer.run() 
        
        
        
        

        
            
            
            
obj = NonBlockVisualizer()

velo_pcds = sorted(os.listdir(dir_path))
label_frames = sorted(os.listdir(label_path)) 

print("len pcd =", len(velo_pcds))
print("len label = ", len(label_frames))
for i in range(len(velo_pcds)):
       pcd_file = velo_pcds[i]
       label = label_frames[i] 
       label_frame_path = os.path.join(label_path, label)
       label_file = open(label_frame_path, "r")
       label_f = label_file.read() 
       labels = json.loads(label_f) 
       
     
    #for filename in sorted(os.listdir(pcd_path)):
    #   size_float = 4
    #   list_pcd = []
       bboxes = []
       for l in range(len(labels)):
          obj_id = labels[l]["obj_id"] 
          obj_type = labels[l]["obj_type"] 
          x = labels[l]["psr"]["position"]["x"]
          y = labels[l]["psr"]["position"]["y"] 
          z = labels[l]["psr"]["position"]["z"] 
          rot_x, rot_y, rot_z = labels[l]["psr"]["rotation"]["x"], labels[l]["psr"]["rotation"]["y"], labels[l]["psr"]["rotation"]["z"] 
          center = [x, y, z] 
          scale_l = labels[l]["psr"]["scale"]["x"]
          scale_w = labels[l]["psr"]["scale"]["y"]
          scale_h = labels[l]["psr"]["scale"]["z"] 
          
          bboxes.append([center, scale_l, scale_w, scale_h, rot_z, obj_type])
          
           
          
       if pcd_file.endswith(".pcd"):
    #       # Load point cloud file
    #       file_path = os.path.join(pcd_path, filename)
    #       with open(file_path, "rb") as f:
    #          byte = f.read(size_float * 4)
    #          while byte:
    #              x, y, z, intensity = struct.unpack("ffff", byte)
    #              list_pcd.append([x, y, z])
    #              byte = f.read(size_float * 4)
    #  
    #       
    #       points = np.asarray(list_pcd)
        
           # Load point cloud file
           pcd = o3d.io.read_point_cloud(os.path.join(dir_path, pcd_file))
        
        
        
  
           #pcd = o3d.geometry.PointCloud()
           #pcd.points = o3d.utility.Vector3dVector(points)
           
           
           
           obj.update_renderer(pcd, bboxes)
           
           time.sleep(0.5)
        

        
       
            

