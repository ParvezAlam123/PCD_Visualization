import torch 
import torch.nn as nn 
import numpy as np 
import json 
import os 
import open3d as o3d
import time
import matplotlib.pyplot as plt 
import struct 
from pyquaternion import Quaternion
import copy 


class Nuscene():
    def __init__(self, root):
        self.root = root 
        self.table_names = ['attribute', 'calibrated_sensor', 'category', 'ego_pose', 
                          'instance', 'log', 'map', 'sample', 'sample_annotation', 'sample_data', 'scene', 'sensor', 'visibility']
        self.attribute = self.read_table('attribute.json')
        self.calibrated_sensor = self.read_table('calibrated_sensor.json')
        self.category = self.read_table('category.json')
        self.ego_pose = self.read_table('ego_pose.json')
        self.instance = self.read_table('instance.json')
        self.log = self.read_table('log.json')
        self.map = self.read_table('map.json')
        self.sample = self.read_table('sample.json')
        self.sample_annotation = self.read_table('sample_annotation.json')
        self.sample_data = self.read_table('sample_data.json')
        self.scene = self.read_table('scene.json')
        self.sensor = self.read_table('sensor.json')
        self.visibility = self.read_table('visibility.json')

        self.token2ind = self.token2ind()
        self.sample_decorate() 





    def read_table(self, table_name):
        path = os.path.join(self.root, table_name)
        f = open(path, 'r')
        file = f.read() 
        table = json.loads(file)
        return table 
    

    def token2ind(self):
        token2ind = {}
        for i in range(len(self.table_names)):
            token2ind[self.table_names[i]] = {}

        for i in range(len(self.attribute)):
            token2ind['attribute'][self.attribute[i]['token']] = i 

        for i in range(len(self.calibrated_sensor)):
            token2ind['calibrated_sensor'][self.calibrated_sensor[i]['token']] = i 

        for i in range(len(self.category)):
            token2ind['category'][self.category[i]['token']] = i 

        for i in range(len(self.ego_pose)):
            token2ind['ego_pose'][self.ego_pose[i]['token']] = i 

        for i in range(len(self.instance)):
            token2ind['instance'][self.instance[i]['token']] = i 

        for i in range(len(self.log)):
            token2ind['log'][self.log[i]['token']] = i 

        for i in range(len(self.map)):
            token2ind['map'][self.map[i]['token']] = i 

        for i in range(len(self.sample)):
            token2ind['sample'][self.sample[i]['token']] = i 

        for i in range(len(self.sample_annotation)):
            token2ind['sample_annotation'][self.sample_annotation[i]['token']] = i 

        for i in range(len(self.sample_data)):
            token2ind['sample_data'][self.sample_data[i]['token']] = i 

        for i in range(len(self.scene)):
            token2ind['scene'][self.scene[i]['token']] = i 

        for i in range(len(self.sensor)):
            token2ind['sensor'][self.sensor[i]['token']] = i 

        for i in range(len(self.visibility)):
            token2ind['visibility'][self.visibility[i]['token']] = i 

        
        return token2ind 
    
    
    def get(self, table_name, token):
        
        if table_name == 'attribute':
            return self.attribute[self.token2ind['attribute'][token]]
        
        if table_name == 'calibrated_sensor':
            return self.calibrated_sensor[self.token2ind['calibrated_sensor'][token]]
        
        if table_name == 'category':
            return self.category[self.token2ind['category'][token]]
        
        if table_name == 'ego_pose':
            return self.ego_pose[self.token2ind['ego_pose'][token]]
        
        if table_name == 'instance':
            return self.instance[self.token2ind['instance'][token]]
        
        if table_name == 'log':
            return self.log[self.token2ind['log'][token]]
        
        if table_name == 'map':
            return self.map[self.token2ind['map'][token]]
        
        if table_name == 'sample':
            return self.sample[self.token2ind['sample'][token]]
        
        if table_name == 'sample_annotation':
            return self.sample_annotation[self.token2ind['sample_annotation'][token]]
        
        if table_name == 'sample_data':
            return self.sample_data[self.token2ind['sample_data'][token]]
        
        if table_name == 'scene':
            return self.scene[self.token2ind['scene'][token]]
        
        if table_name == 'sensor':
            return self.sensor[self.token2ind['sensor'][token]]
        
        if table_name == 'visibility':
            return self.visibility[self.token2ind['visibility'][token]]
        


    def sample_decorate(self):

        # Decorate(add short-cut) sample_annotation table with for category_name 
        for record in self.sample_annotation:
            inst = self.get('instance', record['instance_token'])
            record['category_name'] = self.get('category', inst['category_token'])['name']

        # Decorate (add short-cut) sample_data with sensor information
        for record in self.sample_data:
            cs_record = self.get('calibrated_sensor', record['calibrated_sensor_token'])
            sensor_record = self.get('sensor', cs_record['sensor_token'])
            record['sensor_modality'] = sensor_record['modality']
            record['channel'] = sensor_record['channel']

        # Reverse index sample with sample_data and annotation 
        for record in self.sample:
            record['data'] = {}
            record['anns'] = [] 

        for record in self.sample_data:
            if record['is_key_frame']:
                sample_record = self.get('sample', record['sample_token'])
                sample_record['data'][record['channel']] = record['token']

        
        for ann_record in self.sample_annotation:
            sample_record = self.get('sample', ann_record['sample_token'])
            sample_record['anns'].append(ann_record['token'])

        






global lines 
lines = [] 
 




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
               track_id = dim_of_bb[5] 
        
               #self.__visualizer.add_geometry(line_set)
               #lines.append(line_set)
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices)
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               
               if track_id == 0:
                  colors = [[0, 0, 0.2] for i in range(len(edges))] 
                
               if  track_id == 1:
                  colors = [[0, 0, 0.4] for i in range(len(edges))] 
                  
               if track_id == 2:
                  colors = [[0, 0, 0.6] for i in range(len(edges))] 
                  
               if track_id == 3:
                  colors = [[0, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 4 :
                  colors = [[0, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 5:
                  colors = [[0, 0.2, 0.2] for i in range(len(edges))] 
               
               if track_id == 6:
                  colors = [[0, 0.2, 0.4] for i in range(len(edges))] 
                  
               if track_id == 7:
                  colors = [[0, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 8:
                  colors = [[0, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 9:
                  colors = [[0, 0.2, 1] for i in range(len(edges))] 
                  
               if track_id == 10:
                  colors = [[0, 0.4, 0.2] for i in range(len(edges))]
                  
               if track_id == 11:
                  colors = [[0, 0.4, 0.4] for i in range(len(edges))]
                  
               if track_id == 12:
                  colors = [[0, 0.4, 0.6] for i in range(len(edges))] 
                  
               if track_id == 13:
                  colors = [[0, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 14:
                  colors = [[0, 0.4, 1] for i in range(len(edges))] 
                  
               if track_id == 15:
                  colors = [[0, 0.6, 0.2] for i in range(len(edges))] 
                  
               if track_id == 16:
                  colors = [[0, 0.6, 0.4] for i in range(len(edges))] 
                  
               if track_id == 17:
                  colors = [[0, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 18:
                  colors = [[0, 0.6, 1] for i in range(len(edges))] 
                  
               if track_id == 19:
                  colors = [[0, 0.8, 0.2] for i in range(len(edges))] 
                  
               if track_id == 20:
                  colors = [[0, 0.8, 0.4] for i in range(len(edges))]
                  
               if track_id == 21:
                  colors = [[0, 0.8, 0.6] for i in range(len(edges))] 
                  
               if track_id == 22:
                  colors = [[0, 0.8, 0.8] for i in range(len(edges))] 
                  
               if track_id == 23:
                  colors = [[0, 0.8, 1] for i in range(len(edges))] 
                  
               if track_id == 24:
                  colors = [[0, 1, 0] for i in range(len(edges))] 
                  
               if track_id == 25:
                  colors = [[0, 1, 0.2] for i in range(len(edges))] 
                  
               if track_id == 26 :
                  colors = [[0, 1, 0.4] for i in range(len(edges))] 
                  
               if track_id == 27 :
                  colors = [[0, 1, 0.6] for i in range(len(edges))] 
                  
               if track_id == 28 :
                  colors = [[0, 1, 0.8] for i in range(len(edges))] 
                  
               if track_id == 29 :
                  colors = [[0, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 30:
                  colors = [[0.2, 0, 0] for i in range(len(edges))] 
               
               if track_id == 31 :
                  colors = [[0.2, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 32:
                  colors = [[0.2, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 33 :
                  colors = [[0.2, 0, 0.6] for i in range(len(edges))] 
                  
               if track_id == 34 :
                  colors = [[0.2, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 35 :
                  colors = [[0.2, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 36:
                  colors = [[0.2, 0.2, 0] for i in range(len(edges))]
                  
               if track_id == 37 :
                  colors = [[0.2, 0.2, 0.2] for i in range(len(edges))] 
                  
               if track_id == 38 :
                  colors = [[0.2, 0.2, 0.4] for i in range(len(edges))]
                  
               
               if track_id == 39:
                  colors = [[0.2, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 40:
                  colors = [[0.2, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 41:
                  colors = [[0.2, 0.2, 1] for i in range(len(edges))]
                  
               if track_id == 42:
                  colors = [[0.2, 0.4, 0] for i in range(len(edges))] 
                  
               if track_id == 43:
                  colors = [[0.2, 0.4, 0.2] for i in range(len(edges))] 
                  
               if track_id == 44:
                  colors = [[0.2, 0.4, 0.4] for i in range(len(edges))]
                  
               if track_id == 45:
                  colors = [[0.2, 0.4, 0.6] for i in range(len(edges))]
                  
               if track_id == 46:
                  colors = [[0.2, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 47:
                  colors = [[0.2, 0.4, 1] for i in range(len(edges))]
                  
               if track_id == 48:
                  colors = [[0.2, 0.6, 0] for i in range(len(edges))]
                  
               if track_id == 49:
                  colors = [[0.2, 0.6, 0.2] for i in range(len(edges))]
                  
               if track_id == 50:
                  colors = [[0.2, 0.6, 0.4] for i in range(len(edges))]
                  
               if track_id == 51:
                  colros = [[0.2, 0.6, 0.6] for i in range(len(edges))]
                  
               if track_id == 52:
                  colors = [[0.2, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 53:
                  colors = [[0.2, 0.6, 1] for i in range(len(edges))]
                  
               if track_id == 54:
                  colors = [[0.2, 0.8, 0] for i in range(len(edges))]
                  
               if track_id == 55:
                  colors = [[0.2, 1, 0] for i in range(len(edges))]
                  
               if track_id == 56:
                  colors = [[0.2, 1, 0.2] for i in range(len(edges))]
                  
               if track_id == 56:
                  colors = [[0.2, 1, 0.4] for i in range(len(edges))] 
                  
               if track_id == 57:
                  colors = [[0.2, 1, 0.6] for i in range(len(edges))]
                  
               if track_id == 58:
                  colors = [[0.2, 1, 0.8] for i in range(len(edges))] 
                  
               if track_id == 59:
                  colors = [[0.2, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 60:
                  colors = [[0.4, 0, 0] for i in range(len(edges))]
                  
               if track_id == 61:
                  colors = [[0.4, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 62:
                  colors = [[0.4, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 63:
                  colors = [[0.4, 0, 0.6] for i in range(len(edges))]
                  
               if track_id == 64:
                  colors = [[0.4, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 65:
                  colors = [[0.4, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 66:
                  colors = [[0.4, 0.2, 0] for i in range(len(edges))]
                  
               if track_id == 67:
                  colors = [[0.4, 0.2, 0.2] for i in range(len(edges))]
                  
               if track_id == 68:
                  colors = [[0.4, 0.2, 0.4] for i in range(len(edges))]
                  
               if track_id == 69:
                  colors = [[0.4, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 70:
                  colors = [[0.4, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 71:
                  colors = [[0.4, 0.2, 1] for i in range(len(edges))] 
                  
               if track_id == 72:
                  colors = [[0.4, 0.4, 0] for i in range(len(edges))] 
                  
               if track_id == 73:
                  colors = [[0.4, 0.4, 0.2] for i in range(len(edges))] 
                  
               if track_id == 74:
                  colors = [[0.4, 0.4, 0.4] for i in range(len(edges))] 
                  
               if track_id == 75:
                  colors = [[0.4, 0.4, 0.6] for i in range(len(edges))]
                  
               if track_id == 76:
                  colors = [[0.4, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 77:
                  colors = [[0.4, 0.4, 1] for i in range(len(edges))]
                  
               if track_id == 78:
                  colors = [[0.4, 0.6, 0] for i in range(len(edges))]
                  
               if track_id == 79:
                  colors = [[0.4, 0.6, 0.2] for i in range(len(edges))]
                  
               if track_id == 80:
                  colors = [[0.4, 0.6, 0.4] for i in range(len(edges))] 
                  
               if track_id == 81:
                  colors = [[0.4, 0.6, 0.6] for i in range(len(edges))]
                  
               if track_id == 82:
                  colors = [[0.4, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 83:
                  colors = [[0.4, 0.6, 1] for i in range(len(edges))]
                  
               if track_id == 84:
                  colors = [[0.4, 0.8, 0] for i in range(len(edges))]
                  
               if track_id == 85:
                  colors = [[0.4, 0.8, 0.2] for i in range(len(edges))]
                  
               if track_id == 86:
                  colors = [[0.4, 0.8, 0.4] for i in range(len(edges))]
                  
               if track_id == 87:
                  colors = [[0.4, 0.8, 0.6] for i in range(len(edges))] 
                  
               if track_id == 88:
                  colors = [[0.4, 0.8, 0.8] for i in range(len(edges))]
                  
               if track_id == 89:
                  colors = [[0.4, 0.8, 1] for i in range(len(edges))]
                  
               if track_id == 90:
                  colors = [[0.4, 1, 0] for i in range(len(edges))]
                  
               if track_id == 91:
                  colors = [[0.4, 1, 0.2] for i in range(len(edges))] 
                  
               if track_id == 92:
                  colors = [[0.4, 1, 0.4] for i in range(len(edges))]
                  
               if track_id == 93:
                  colors = [[0.4, 1, 0.6] for i in range(len(edges))]
                  
               if track_id == 94:
                  colors = [[0.4, 1, 0.8] for i in range(len(edges))]
                  
               if track_id == 95:
                  colors = [[0.4, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 96:
                  colors = [[0.6, 0, 0] for i in range(len(edges))]
                  
               if track_id == 97:
                  colors = [[0.6, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 98:
                  colors = [[0.6, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 99:
                  colors = [[0.6, 0, 0.6] for i in range(len(edges))]
                  
               if track_id == 100:
                  colors = [[0.6, 0, 0.8] for i in range(len(edges))]
                  
                  
             
                 
                        
                  
                  
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
               track_id = dim_of_bb[5] 
               
               
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices)
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               
               if track_id == 0:
                  colors = [[0, 0, 0.2] for i in range(len(edges))] 
                
               if  track_id == 1:
                  colors = [[0, 0, 0.4] for i in range(len(edges))] 
                  
               if track_id == 2:
                  colors = [[0, 0, 0.6] for i in range(len(edges))] 
                  
               if track_id == 3:
                  colors = [[0, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 4 :
                  colors = [[0, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 5:
                  colors = [[0, 0.2, 0.2] for i in range(len(edges))] 
               
               if track_id == 6:
                  colors = [[0, 0.2, 0.4] for i in range(len(edges))] 
                  
               if track_id == 7:
                  colors = [[0, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 8:
                  colors = [[0, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 9:
                  colors = [[0, 0.2, 1] for i in range(len(edges))] 
                  
               if track_id == 10:
                  colors = [[0, 0.4, 0.2] for i in range(len(edges))]
                  
               if track_id == 11:
                  colors = [[0, 0.4, 0.4] for i in range(len(edges))]
                  
               if track_id == 12:
                  colors = [[0, 0.4, 0.6] for i in range(len(edges))] 
                  
               if track_id == 13:
                  colors = [[0, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 14:
                  colors = [[0, 0.4, 1] for i in range(len(edges))] 
                  
               if track_id == 15:
                  colors = [[0, 0.6, 0.2] for i in range(len(edges))] 
                  
               if track_id == 16:
                  colors = [[0, 0.6, 0.4] for i in range(len(edges))] 
                  
               if track_id == 17:
                  colors = [[0, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 18:
                  colors = [[0, 0.6, 1] for i in range(len(edges))] 
                  
               if track_id == 19:
                  colors = [[0, 0.8, 0.2] for i in range(len(edges))] 
                  
               if track_id == 20:
                  colors = [[0, 0.8, 0.4] for i in range(len(edges))]
                  
               if track_id == 21:
                  colors = [[0, 0.8, 0.6] for i in range(len(edges))] 
                  
               if track_id == 22:
                  colors = [[0, 0.8, 0.8] for i in range(len(edges))] 
                  
               if track_id == 23:
                  colors = [[0, 0.8, 1] for i in range(len(edges))] 
                  
               if track_id == 24:
                  colors = [[0, 1, 0] for i in range(len(edges))] 
                  
               if track_id == 25:
                  colors = [[0, 1, 0.2] for i in range(len(edges))] 
                  
               if track_id == 26 :
                  colors = [[0, 1, 0.4] for i in range(len(edges))] 
                  
               if track_id == 27 :
                  colors = [[0, 1, 0.6] for i in range(len(edges))] 
                  
               if track_id == 28 :
                  colors = [[0, 1, 0.8] for i in range(len(edges))] 
                  
               if track_id == 29 :
                  colors = [[0, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 30:
                  colors = [[0.2, 0, 0] for i in range(len(edges))] 
               
               if track_id == 31 :
                  colors = [[0.2, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 32:
                  colors = [[0.2, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 33 :
                  colors = [[0.2, 0, 0.6] for i in range(len(edges))] 
                  
               if track_id == 34 :
                  colors = [[0.2, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 35 :
                  colors = [[0.2, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 36:
                  colors = [[0.2, 0.2, 0] for i in range(len(edges))]
                  
               if track_id == 37 :
                  colors = [[0.2, 0.2, 0.2] for i in range(len(edges))] 
                  
               if track_id == 38 :
                  colors = [[0.2, 0.2, 0.4] for i in range(len(edges))]
                  
               
               if track_id == 39:
                  colors = [[0.2, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 40:
                  colors = [[0.2, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 41:
                  colors = [[0.2, 0.2, 1] for i in range(len(edges))]
                  
               if track_id == 42:
                  colors = [[0.2, 0.4, 0] for i in range(len(edges))] 
                  
               if track_id == 43:
                  colors = [[0.2, 0.4, 0.2] for i in range(len(edges))] 
                  
               if track_id == 44:
                  colors = [[0.2, 0.4, 0.4] for i in range(len(edges))]
                  
               if track_id == 45:
                  colors = [[0.2, 0.4, 0.6] for i in range(len(edges))]
                  
               if track_id == 46:
                  colors = [[0.2, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 47:
                  colors = [[0.2, 0.4, 1] for i in range(len(edges))]
                  
               if track_id == 48:
                  colors = [[0.2, 0.6, 0] for i in range(len(edges))]
                  
               if track_id == 49:
                  colors = [[0.2, 0.6, 0.2] for i in range(len(edges))]
                  
               if track_id == 50:
                  colors = [[0.2, 0.6, 0.4] for i in range(len(edges))]
                  
               if track_id == 51:
                  colros = [[0.2, 0.6, 0.6] for i in range(len(edges))]
                  
               if track_id == 52:
                  colors = [[0.2, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 53:
                  colors = [[0.2, 0.6, 1] for i in range(len(edges))]
                  
               if track_id == 54:
                  colors = [[0.2, 0.8, 0] for i in range(len(edges))]
                  
               if track_id == 55:
                  colors = [[0.2, 1, 0] for i in range(len(edges))]
                  
               if track_id == 56:
                  colors = [[0.2, 1, 0.2] for i in range(len(edges))]
                  
               if track_id == 56:
                  colors = [[0.2, 1, 0.4] for i in range(len(edges))] 
                  
               if track_id == 57:
                  colors = [[0.2, 1, 0.6] for i in range(len(edges))]
                  
               if track_id == 58:
                  colors = [[0.2, 1, 0.8] for i in range(len(edges))] 
                  
               if track_id == 59:
                  colors = [[0.2, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 60:
                  colors = [[0.4, 0, 0] for i in range(len(edges))]
                  
               if track_id == 61:
                  colors = [[0.4, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 62:
                  colors = [[0.4, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 63:
                  colors = [[0.4, 0, 0.6] for i in range(len(edges))]
                  
               if track_id == 64:
                  colors = [[0.4, 0, 0.8] for i in range(len(edges))] 
                  
               if track_id == 65:
                  colors = [[0.4, 0, 1] for i in range(len(edges))] 
                  
               if track_id == 66:
                  colors = [[0.4, 0.2, 0] for i in range(len(edges))]
                  
               if track_id == 67:
                  colors = [[0.4, 0.2, 0.2] for i in range(len(edges))]
                  
               if track_id == 68:
                  colors = [[0.4, 0.2, 0.4] for i in range(len(edges))]
                  
               if track_id == 69:
                  colors = [[0.4, 0.2, 0.6] for i in range(len(edges))]
                  
               if track_id == 70:
                  colors = [[0.4, 0.2, 0.8] for i in range(len(edges))]
                  
               if track_id == 71:
                  colors = [[0.4, 0.2, 1] for i in range(len(edges))] 
                  
               if track_id == 72:
                  colors = [[0.4, 0.4, 0] for i in range(len(edges))] 
                  
               if track_id == 73:
                  colors = [[0.4, 0.4, 0.2] for i in range(len(edges))] 
                  
               if track_id == 74:
                  colors = [[0.4, 0.4, 0.4] for i in range(len(edges))] 
                  
               if track_id == 75:
                  colors = [[0.4, 0.4, 0.6] for i in range(len(edges))]
                  
               if track_id == 76:
                  colors = [[0.4, 0.4, 0.8] for i in range(len(edges))] 
                  
               if track_id == 77:
                  colors = [[0.4, 0.4, 1] for i in range(len(edges))]
                  
               if track_id == 78:
                  colors = [[0.4, 0.6, 0] for i in range(len(edges))]
                  
               if track_id == 79:
                  colors = [[0.4, 0.6, 0.2] for i in range(len(edges))]
                  
               if track_id == 80:
                  colors = [[0.4, 0.6, 0.4] for i in range(len(edges))] 
                  
               if track_id == 81:
                  colors = [[0.4, 0.6, 0.6] for i in range(len(edges))]
                  
               if track_id == 82:
                  colors = [[0.4, 0.6, 0.8] for i in range(len(edges))] 
                  
               if track_id == 83:
                  colors = [[0.4, 0.6, 1] for i in range(len(edges))]
                  
               if track_id == 84:
                  colors = [[0.4, 0.8, 0] for i in range(len(edges))]
                  
               if track_id == 85:
                  colors = [[0.4, 0.8, 0.2] for i in range(len(edges))]
                  
               if track_id == 86:
                  colors = [[0.4, 0.8, 0.4] for i in range(len(edges))]
                  
               if track_id == 87:
                  colors = [[0.4, 0.8, 0.6] for i in range(len(edges))] 
                  
               if track_id == 88:
                  colors = [[0.4, 0.8, 0.8] for i in range(len(edges))]
                  
               if track_id == 89:
                  colors = [[0.4, 0.8, 1] for i in range(len(edges))]
                  
               if track_id == 90:
                  colors = [[0.4, 1, 0] for i in range(len(edges))]
                  
               if track_id == 91:
                  colors = [[0.4, 1, 0.2] for i in range(len(edges))] 
                  
               if track_id == 92:
                  colors = [[0.4, 1, 0.4] for i in range(len(edges))]
                  
               if track_id == 93:
                  colors = [[0.4, 1, 0.6] for i in range(len(edges))]
                  
               if track_id == 94:
                  colors = [[0.4, 1, 0.8] for i in range(len(edges))]
                  
               if track_id == 95:
                  colors = [[0.4, 1, 1] for i in range(len(edges))] 
                  
               if track_id == 96:
                  colors = [[0.6, 0, 0] for i in range(len(edges))]
                  
               if track_id == 97:
                  colors = [[0.6, 0, 0.2] for i in range(len(edges))]
                  
               if track_id == 98:
                  colors = [[0.6, 0, 0.4] for i in range(len(edges))]
                  
               if track_id == 99:
                  colors = [[0.6, 0, 0.6] for i in range(len(edges))]
                  
               if track_id == 100:
                  colors = [[0.6, 0, 0.8] for i in range(len(edges))]
                  
                  
             
                 
               
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
        
        
        



class Box:
   """ Simple data class representing a 3D box including label, score and velocity """     
   def __init__(self, center, size, orientation):
   
      assert type(center) == np.ndarray
      assert len(size) == 3
      assert type(orientation) == Quaternion
      
      
      self.center = center 
      self.size = size
      self.orientation = orientation
      
   def translate(self, x):
      
      self.center += x
      
   def rotate(self, orientation):
      self.center = np.dot(orientation.rotation_matrix, self.center)
      self.orientation = orientation * self.orientation  
      
           


def quaternion_to_angle(quaternion):
    # Extract the scalar and vector components of the quaternion
    w, x, y, z = quaternion
    
    # Calculate the squared magnitude of the vector component
    magnitude_squared = x**2 + y**2 + z**2
    
    # If the magnitude is zero, return zero angle
    if magnitude_squared == 0:
        return 0.0
    
    # Calculate the angle using the arctangent function and return it in radians
    angle = 2.0 * np.arctan2(np.sqrt(magnitude_squared), w)
    return angle
    

def convert_kitti_bin_to_pcd(binFilePath):
    if binFilePath.endswith('.bin'):
    
       size_float = 4
       list_pcd = []
       with open(binFilePath, "rb") as f:
           byte = f.read(size_float * 5)
           while byte:
              x, y, z , intensity, ring_index = struct.unpack("fffff", byte)
              list_pcd.append([x, y, z])
              byte = f.read(size_float * 5)
       np_pcd = np.asarray(list_pcd)
       pcd = o3d.geometry.PointCloud()
       pcd.points = o3d.utility.Vector3dVector(np_pcd)
       return pcd



nusc = Nuscene("/media/parvez_alam/Expansion/Nuscene_US/Train/Metadata/v1.0-trainval_meta/v1.0-trainval")

obj = NonBlockVisualizer() 
data_path = '/media/parvez_alam/Expansion/Nuscene_US/Train/v1.0-trainval01_blobs'


past_frames_instances = [] 
past_frame = 1 
for i in range(len(nusc.scene)):
   my_scene = nusc.scene[i] 
   flag = 1
   first_flag = 1
   n_frame = 0 
   
   #print("scene table = ", my_scene) 
   instance_id_db = []              #for storing instance id
   track_id_db = [] 
   id_dict = {} 
   id = 0
   #past_frame = 1 
   #past_frame_instances = [] 
   while(flag==1):
      current_frame_instances = []
      n_frame = n_frame + 1
      if first_flag == 1:
         first_sample_token = my_scene['first_sample_token']
         last_sample_token = my_scene['last_sample_token'] 
   
         my_sample = nusc.get('sample', first_sample_token) 
         next_sample = my_sample['next']
         first_flag=0
      else:
         my_sample = nusc.get('sample', first_sample_token)
         next_sample = my_sample['next']
         
         #print("sample table = ", my_sample) 
   
      
      
      # get sample lidar data
      sensor = 'LIDAR_TOP'
      lidar_data = nusc.get('sample_data', my_sample['data'][sensor]) 
      file_name = lidar_data['filename']
      
      
      # get front camera data 
      sensor = 'CAM_FRONT'
      camera_data = nusc.get('sample_data', my_sample['data'][sensor])
      cam_File = camera_data['filename']
      
      
      
      file_path = os.path.join(data_path, file_name)
   
      pcd = convert_kitti_bin_to_pcd(file_path)
      
      # Retrive sensor and ego  pose record
      cs_record = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
      sensor_record = nusc.get('sensor', cs_record['sensor_token'])
      pose_record = nusc.get('ego_pose', lidar_data['ego_pose_token'])
      
      
      
      annotation_token_list = my_sample['anns']
      bb_list = []
      if past_frame:
         for k in range(len(annotation_token_list)):
            annotation_token = annotation_token_list[k] 
            ann_record = nusc.get('sample_annotation', annotation_token) 
            instance_id = ann_record['instance_token']
            category = ann_record['category_name'] 
            visibility_token = ann_record['visibility_token']
            center = ann_record['translation']
            size = ann_record['size']
            rotation = ann_record['rotation']
            box = Box(np.array(center), size, Quaternion(rotation))
         
         
         
         
            # Move box to ego vehicle coordinate system
            yaw = Quaternion(pose_record['rotation']).yaw_pitch_roll[0]
            box.translate(-np.array(pose_record['translation']))
            box.rotate(Quaternion(scalar=np.cos(yaw/2), vector=[0, 0, np.sin(yaw/2)]).inverse)
         
            # Move box to sensor coordinate system 
            box.translate(-np.array(cs_record['translation']))
            #box.rotate(Quaternion(cs_record['rotation']).inverse)
            yaw = Quaternion(cs_record['rotation']).yaw_pitch_roll[0]
            box.rotate(Quaternion(scalar=np.cos(yaw/2), vector=[0,0,np.sin(yaw/2)]).inverse)
         
         
         
            rotation_angle = quaternion_to_angle(box.orientation) 
            
            if instance_id not in instance_id_db :
               instance_id_db.append(instance_id) 
               track_id_db.append(id) 
               id_dict[instance_id] = id 
               id = id + 1   
               past_frames_instances.append(instance_id) 
               
            track_id = id_dict[instance_id] 
            dim_of_box = [box.center, size[1], size[0], size[2], rotation_angle, track_id] 
            bb_list.append(dim_of_box)
         
      else:
         for j in range(len(annotation_token_list)):
               ann_token = annotation_token_list[j]
               annotation_record = nusc.get('sample_annotation', ann_token)
               current_frame_instances.append(annotation_record['instance_token'])  
               
         for m in range(len(past_frames_instances)):
                if past_frames_instances[m] not in current_frame_instances :
                   for j in range(len(instance_id_db)):
                      if past_frames_instances[m] == instance_id_db[j]:
                         track_id = id_dict[past_frames_instances[m]]
                         del id_dict[past_frames_instances[m]]
                         del instance_id_db[j] 
                         for n in range(len(track_id_db)):
                            if track_id == track_id_db[n]:
                               del track_id_db[n]
                               break 
                         print("deletion done")
                         break 
                   
         
         
         print("track id = ", track_id_db)
         print("track id db len = ", len(track_id_db))
         print("current instance = ", len(current_frame_instances))
         #print("current_instance = ", current_frame_instances)
         print("past_instance = ", len(past_frames_instances)) 
                           
         
         for k in range(len(annotation_token_list)):
            annotation_token = annotation_token_list[k] 
            ann_record = nusc.get('sample_annotation', annotation_token) 
            instance_id = ann_record['instance_token']
            category = ann_record['category_name'] 
            visibility_token = ann_record['visibility_token']
            center = ann_record['translation']
            size = ann_record['size']
            rotation = ann_record['rotation']
            box = Box(np.array(center), size, Quaternion(rotation))
         
         
         
         
            # Move box to ego vehicle coordinate system
            yaw = Quaternion(pose_record['rotation']).yaw_pitch_roll[0]
            box.translate(-np.array(pose_record['translation']))
            box.rotate(Quaternion(scalar=np.cos(yaw/2), vector=[0, 0, np.sin(yaw/2)]).inverse)
         
            # Move box to sensor coordinate system 
            box.translate(-np.array(cs_record['translation']))
            #box.rotate(Quaternion(cs_record['rotation']).inverse)
            yaw = Quaternion(cs_record['rotation']).yaw_pitch_roll[0]
            box.rotate(Quaternion(scalar=np.cos(yaw/2), vector=[0,0,np.sin(yaw/2)]).inverse)
         
         
         
            rotation_angle = quaternion_to_angle(box.orientation) 
            
         
            
             
            
            if instance_id not in instance_id_db:
               if instance_id_db == [] :
                  instance_id_db.append(instance_id) 
                  track_id_db.append(0) 
                  id_dict[instance_id] = 0  
                  
               else:
                  sorted_track_ids = sorted(track_id_db) 
                  if min(sorted_track_ids) > 0 :
                     minimum_id = min(sorted_track_ids)
                     instance_id_db.append(instance_id) 
                     track_id_db.append(minimum_id - 1)
                     id_dict[instance_id] = minimum_id - 1 
                     
                  else:     
                     for j in range(len(sorted_track_ids)):
                        if j+1 < len(sorted_track_ids):
                           if sorted_track_ids[j] + 1  != sorted_track_ids[j+1]:
                              instance_id_db.append(instance_id) 
                              track_id_db.append(sorted_track_ids[j] + 1) 
                              id_dict[instance_id] = sorted_track_ids[j] + 1 
                              break 
                     
                           
                     if j+1 == len(sorted_track_ids):
                        id = sorted_track_ids[-1] + 1 
                        instance_id_db.append(instance_id) 
                        track_id_db.append(id) 
                        id_dict[instance_id] = id 
                     
                     
         #past_frame_instaces = current_frame_instances 
                      
                              
            
            track_id = id_dict[instance_id] 
            dim_of_box = [box.center, size[1], size[0], size[2], rotation_angle, track_id] 
            bb_list.append(dim_of_box)
         
     
      
      print("track id after = ", track_id_db)  
      print("id db len after =", len(track_id_db))
      print("\n")
      
      if past_frame == 0:
         past_frames_instances = current_frame_instances 
         
      
      past_frame = 0
      #if n_frame == 10:
      #print("bb shape = ", bb_list)
      obj.update_renderer(pcd, bb_list)
      
      
      time.sleep(0.1)
      
      
      
      
      first_sample_token = next_sample 
      
      if next_sample == '':
         flag=0 
         past_frame = 0
                        
         
         


       
   




        
