import numpy as np 
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





class LaserScan:
    """ class that contains laserscan x y, z, r"""

    EXTENSIONS_SCAN = [".bin"]

    def __init__(self, project=False, H=32, W=1024, fov_up=10, fov_down=-30):
        self.project = project 
        self.H = H 
        self.W = W 
        self.proj_fov_up = fov_up 
        self.proj_fov_down = fov_down 
        self.reset()



    def reset(self):
        """ Reset scan members""" 
        self.points = np.zeros((0, 3), dtype=np.float32)     # [N, 3]
        self.remissions = np.zeros((0, 1), dtype=np.float32) # [N, 1]

        # projected range image [H, W] (-1 is no data)
        self.proj_range = np.full((self.H, self.W), -1, dtype=np.float32)

        # unprojected range -(list of depth for each point)
        self.unproj_range = np.zeros((0, 1), dtype=np.float32)

        # projected point cloud xyz-[h,w,3] (-1 means no data)
        self.proj_xyz = np.full((self.H, self.W, 3), -1, dtype=np.float32)

        # projected remission - [H, W] (-1 means no data)
        self.proj_remission = np.full((self.H, self.W), -1, dtype=np.float32)

        # projected index (for each pixel in range image what I am in point cloud); [H, W] (-1 means no data)
        self.proj_idx = np.full((self.H, self.W), -1, dtype=np.int32) 

        # for each point where it is in range image [N, 1]
        self.proj_x = np.zeros((0, 1), dtype=np.float32)
        self.proj_y = np.zeros((0, 1), dtype=np.float32)


        # mask containing for each pixel, if it contains a point or not 
        self.proj_mask = np.zeros((self.H, self.W), dtype=np.int32)

    
    def size(self):
        """ return the size of the point cloud"""
        return self.points.shape[0]
    
    def __len__(self):
        return self.size() 
    

    def open_scan(self, filename):
        """ open raw scan and fill attributes values"""

        # reset just in case any  structure is open 
        self.reset() 

        # check the filename is proper string type 
        if not isinstance(filename, str):
            raise TypeError("Filename should be string type but found {}".format(str(type(filename))))
        
        # check extension is a laser scan 
        if not any(filename.endswith(ext) for ext in self.EXTENSIONS_SCAN):
            raise RuntimeError("Filename extension is not valid laser scan")
        
        # if all is well open laser scan 
        scan = np.fromfile(filename, dtype=np.float32)
        scan = scan.reshape((-1, 5))

        # put in  attribute 
        points = scan[:, 0:3]
        remissions = scan[:, 3:4]
        
        self.set_points(points, remissions)


    def set_points(self,points, remissions=None):
        """ Set scan attribute instead of opening it"""

        # reset any open structure 
        self.reset() 

        # check scan makes sense 
        if not isinstance(points, np.ndarray):
            raise TypeError("Scan should be numpy array")
        
        # check remission make sense 
        if remissions is not None and not isinstance(remissions, np.ndarray):
            raise TypeError("Remissions should be numpy array")
        

        # put the attrubutes 
        self.points = points 
        if self.remissions is not None :
            self.remissions = remissions 
        else:
            self.remissions = np.zeros((points.shape[0]), dtype=np.float32)


        # if projection wanted 
        if self.project:
            self.do_range_projection() 

    
    def do_range_projection(self):
        """ Project a point cloud into a spherical projection image"""

        # laser parameters 
        fov_up = (self.proj_fov_up / 180.0) * np.pi
        fov_down = (self.proj_fov_down / 180.0) * np.pi 
        fov = abs(fov_up) + abs(fov_down)

        # get depth of all the points 
        depth = np.linalg.norm(self.points, 2, axis=1)

        # get scan components 
        scan_x = self.points[:, 0]
        scan_y = self.points[:, 1]
        scan_z = self.points[:, 2]

        # get angle of the projection 
        yaw = np.arctan2(scan_y, scan_x) 
        pitch = np.arcsin(scan_z / depth)

        # get normalized projections 
        proj_x = 0.5 * (yaw / np.pi + 1.0) 
        proj_y = (fov_up - pitch) / fov 
        #for i in range(len(proj_y)):
        #    if proj_y[i] < 0 :
        #        print("pitch = ", pitch[i]*180/np.pi)
        #        print("fov_up=", fov_up*180/np.pi)
        #        break

        # scale to image size using angular resolution 
        proj_x = proj_x * self.W 
        proj_y = proj_y * self.H 

        # round and clamp for use as index 
        proj_x = np.floor(proj_x)
        proj_x = np.minimum(self.W - 1, proj_x)
        proj_x = np.maximum(0, proj_x).astype(np.int32)
        self.proj_x = np.copy(proj_x)    # store a copy in the original order

        proj_y = np.floor(proj_y)
        proj_y = np.minimum(self.H - 1, proj_y)
        proj_y = np.maximum(0, proj_y).astype(np.int32)
        self.proj_y = np.copy(proj_y)   # store a copy in the original order 

        # copy of the depth in original order 
        self.unproj_range = np.copy(depth)


        # order in increaseing depth 
        indices = np.arange(depth.shape[0])
        order = np.argsort(depth)

        depth = depth[order]
        indices = indices[order]
        points = self.points[order]
        remissions = self.remissions[order]
        proj_x = proj_x[order]
        proj_y = proj_y[order]
        
        # assigns to images 
        self.proj_range[proj_y, proj_x] = depth 
        self.proj_xyz[proj_y, proj_x] = points 
        self.proj_remission[proj_y, proj_x] = remissions.flatten() 
        self.proj_idx[proj_y, proj_x] = indices 
        self.proj_mask = (self.proj_idx > 0).astype(np.float32)

 
 
 
 



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
        
        

        
            
            
            




        

        





           


nusc = Nuscene("/media/parvez_alam/Expansion/Nuscene_US/Train/Metadata/v1.0-trainval_meta/v1.0-trainval")

#obj = NonBlockVisualizer() 
data_path = '/media/parvez_alam/Expansion/Nuscene_US/Train/v1.0-trainval01_blobs'



obj = NonBlockVisualizer()






for i in range(700):
   my_scene = nusc.scene[i]
   flag = 1
   first_flag = 1
   n_frame = 0
   while(flag==1):
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
         
   
      
      
      laserscan_obj =  LaserScan(project=True) 
   
      # get sample lidar data
      sensor = 'LIDAR_TOP'
      lidar_data = nusc.get('sample_data', my_sample['data'][sensor])
      file_name = lidar_data['filename']
      
      
      
      
      
      
      file_path = os.path.join(data_path, file_name) 
      laserscan_obj.open_scan(file_path)
      
      
      points = laserscan_obj.proj_xyz
      
      points = np.asarray(points.reshape(-1, 3))
      #print(points.shape)
      
      
      
      pcd = o3d.geometry.PointCloud()
      pcd.points = o3d.utility.Vector3dVector(points)
        
      obj.update_renderer(pcd)
      time.sleep(0.08) 
           
      
      
      first_sample_token = next_sample 
      
      if next_sample == '':
         flag=0
       
   
   
   
    
        






