import torch 
import torch.nn as nn 
import numpy as np 
import json 
import os 
import open3d as o3d
import time
import matplotlib.pyplot as plt 
import struct 





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
            
          
          


nusc = Nuscene("/media/parvez_alam/Expansion/Nuscene_Asia/Train/Metadata/v1.0-trainval_meta/v1.0-trainval")

obj = NonBlockVisualizer() 
data_path = '/media/parvez_alam/Expansion/Nuscene_Asia/Train/v1.0-trainval01_blobs'


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


for i in range(len(nusc.scene)):
   my_scene = nusc.scene[i]
   flag = 1
   first_flag = 1
   while(flag==1):
      if first_flag == 1:
         first_sample_token = my_scene['first_sample_token']
         last_sample_token = my_scene['last_sample_token'] 
   
         my_sample = nusc.get('sample', first_sample_token)
         next_sample = my_sample['next']
         first_flag=0
      else:
         my_sample = nusc.get('sample', first_sample_token)
         next_sample = my_sample['next']
         
   
      
   
   
      sensor = 'LIDAR_TOP'
      lidar_data = nusc.get('sample_data', my_sample['data'][sensor])
      file_name = lidar_data['filename']
   
      file_path = os.path.join(data_path, file_name)
   
      pcd = convert_kitti_bin_to_pcd(file_path)
      
      obj.update_renderer(pcd)
      time.sleep(0.1)
      
      first_sample_token = next_sample 
      
      if next_sample == '':
         flag=0
       
   
   
   
   
   



