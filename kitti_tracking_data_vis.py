import numpy as np 
import open3d as o3d 
import os 
import struct 
import time 






LABEL_DIR = "/home/parvez_alam/Data/Kitti/Tracking/data_tracking_label_2/training/label_02"
POINT_CLOUD_DIR ="/home/parvez_alam/Data/Kitti/Tracking/data_tracking_velodyne/training/velodyne"
CALIB_DIR = "/home/parvez_alam/Data/Kitti/Tracking/data_tracking_calib/training/calib" 
 
calibs = sorted(os.listdir(CALIB_DIR))
scenes = sorted(os.listdir(POINT_CLOUD_DIR))
labels = sorted(os.listdir(LABEL_DIR))
global edges 

edges = np.array([[0, 1], [1,2], [2,3], [3,0],
                 [4,5], [5,6], [6, 7], [7,4],
                 [0,4], [1,5], [2,6], [3, 7]])

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
        
        self.total_bboxes = [] 
        for i in range(100):
           self.total_bboxes.append(o3d.geometry.LineSet())
           
        self.count_bb = 0 
        
        

    def update_renderer(self, pcd, bboxes , wait_time=0):
        self.__pcd_vis.points = pcd.points
        self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
            
            for dim_of_bb in bboxes:
               vertices = dim_of_bb[1] 
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices)
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               colors = [[1, 0, 0] for i in range(len(edges))] 
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
            for dim_of_bb in bboxes:
               vertices = dim_of_bb[1] 
               self.total_bboxes[self.count_bb].points = o3d.utility.Vector3dVector(vertices) 
               self.total_bboxes[self.count_bb].lines = o3d.utility.Vector2iVector(edges) 
               colors = [[1, 0, 0] for i in range(len(edges))]
               self.total_bboxes[self.count_bb].colors = o3d.utility.Vector3dVector(colors) 
               self.count_bb = self.count_bb + 1 
               
            
            for bbox in self.total_bboxes:
               self.__visualizer.update_geometry(bbox) 
               
        
            
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()
        
        
        
        

       
            





def load_kitti_calib(calib_file):
   
   with open(calib_file) as f_calib:
      lines = f_calib.readlines()
      
   P0 = np.array(lines[0].strip('\n').split()[1:], dtype=np.float32)
   P1 = np.array(lines[1].strip('\n').split()[1:], dtype=np.float32)
   P2 = np.array(lines[2].strip('\n').split()[1:], dtype=np.float32)
   P3 = np.array(lines[3].strip('\n').split()[1:], dtype=np.float32)
   R0_rect = np.array(lines[4].strip('\n').split()[1:], dtype=np.float32)
   Tr_velo_to_cam = np.array(lines[5].strip('\n').split()[1:], dtype=np.float32)
   Tr_imu_to_velo = np.array(lines[6].strip('\n').split()[1:], dtype=np.float32) 
   
   return {'P0': P0, 'P1':P1, 'P2':P2, 'P3':P3, 'R0_rect': R0_rect, 'Tr_velo_to_cam': Tr_velo_to_cam.reshape(3,4), 'Tr_imu_to_velo': Tr_imu_to_velo}
   
   
 
 
 
def camera_coordinate_to_point_cloud(box3d, Tr):

   def project_cam2velo(cam, Tr):
      T = np.zeros([4,4], dtype=np.float32)
      T[:3, :] = Tr 
      T[3, 3] = 1 
     
      T_inv = np.linalg.inv(T) 
      lidar_loc_ = np.dot(T_inv, cam) 
      lidar_loc = lidar_loc_[:3]
      
      return lidar_loc.reshape(1,3) 
      
   def ry_to_rz(ry):
      angle = -ry - np.pi / 2
      
      if angle >= np.pi:
         angle -= np.pi 
      if angle < -np.pi:
         angle = 2 * np.pi + angle 
      return angle 
      
      
      
   
   h,w,l,tx,ty,tz,ry = [float(i) for i in box3d]
   cam = np.ones([4,1])
   cam[0] = tx 
   cam[1] = ty
   cam[2] = tz 
   t_lidar = project_cam2velo(cam, Tr) 
  
   
   Box = np.array([[-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
                   [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
                   [0, 0, 0, 0, h, h,  h, h]])
                   
   rz = ry_to_rz(ry) 
   
   rotMat = np.array([[np.cos(rz), -np.sin(rz), 0.0],
                       [np.sin(rz), np.cos(rz), 0.0],
                       [0.0,          0.0,       1.0]])
   
                      
   velo_box = np.dot(rotMat, Box) 
     
   cornerPosInVelo = velo_box + np.tile(t_lidar, (8, 1)).T 
   
   box3d_corner = cornerPosInVelo.transpose() 
   
   return t_lidar, box3d_corner 
   
   
   
     






obj = NonBlockVisualizer() 


for i in range(len(scenes)):
    pcd_file_path = os.path.join(POINT_CLOUD_DIR, scenes[i])
    calib_file = os.path.join(CALIB_DIR, calibs[i]) 
    label_file = os.path.join(LABEL_DIR, labels[i])  
    calibration = load_kitti_calib(calib_file)
    
    
    
    
    # get the total number of frames in particular scene 
    num_frames = len(os.listdir(pcd_file_path)) 
    
    
    bb_list = []            # store bounding boxex of complete scene 
    
    with open(label_file) as f_label:
       lines = f_label.readlines()
       
       for line in lines:
          line = line.strip('\n').split() 
          if line[2] != 'DontCare':
             frame_index = line[0]             # frame number
             center, box3d_corner = camera_coordinate_to_point_cloud(line[10:17], calibration['Tr_velo_to_cam'])
             center = center[0] 
             bb_list.append([frame_index, center, box3d_corner])
          
       
    
    pcd_frames = sorted(os.listdir(pcd_file_path)) 
    
    for n in range(len(pcd_frames)):
       pcd_path = os.path.join(pcd_file_path, pcd_frames[n])
       size_float = 4 
       list_pcd = [] 
       with open(pcd_path, "rb") as f:
          byte = f.read(size_float * 4) 
          while byte:
             x, y, z, intensity = struct.unpack("ffff", byte) 
             list_pcd.append([x,y,z]) 
             byte = f.read(size_float * 4) 
       points = np.asarray(list_pcd) 
       
       
       
       pcd = o3d.geometry.PointCloud() 
       pcd.points = o3d.utility.Vector3dVector(points) 
       
       
       bboxes = [] 
       for k in range(len(bb_list)):
          if int(bb_list[k][0]) == n : 
             bboxes.append([bb_list[k][1], bb_list[k][2]])
             
             
          
       obj.update_renderer(pcd, bboxes)
       
       
       
       
             

       
       
             
       
          
     
          
          
          
          
          
    
       
    
    
    
    
    


