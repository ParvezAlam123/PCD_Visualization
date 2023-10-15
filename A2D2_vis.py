import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 
import struct 
import numpy as np 


cam_fornt_center = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_frontcenter/camera_lidar/20190401_121727/lidar/cam_front_center"
cam_front_left = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_frontleft/camera_lidar/20190401_121727/lidar/cam_front_left"
cam_front_right = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_frontright/camera_lidar/20190401_121727/lidar/cam_front_right"
cam_rear_center = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_rearcenter/camera_lidar/20190401_121727/lidar/cam_rear_center"
cam_side_left = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_sideleft/camera_lidar/20190401_121727/lidar/cam_side_left" 
cam_side_right = "/media/parvez_alam/Expansion/A2D2/Munich/camera_lidar-20190401121727_lidar_sideright/camera_lidar/20190401_121727/lidar/cam_side_right" 





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






cam_fornt_center_files = sorted(os.listdir(cam_fornt_center)) 
cam_front_left_files = sorted(os.listdir(cam_front_left)) 
cam_front_right_files = sorted(os.listdir(cam_front_right)) 
cam_rear_center_files = sorted(os.listdir(cam_rear_center)) 
cam_side_left_files = sorted(os.listdir(cam_side_left)) 
cam_side_right_files = sorted(os.listdir(cam_side_right)) 





for i in  range(100):
        
           cam_fornt_center_files_path = os.path.join(cam_fornt_center, cam_fornt_center_files[i]) 
           lidar_data = np.load(cam_fornt_center_files_path ) 
           points1 = np.array(lidar_data['pcloud_points']) 
           
           cam_front_left_files_path = os.path.join(cam_front_left, cam_front_left_files[i]) 
           lidar_data = np.load(cam_front_left_files_path)
           points2 = np.array(lidar_data['pcloud_points']) 
           
           cam_front_right_files_path = os.path.join(cam_front_right, cam_front_right_files[i]) 
           lidar_data = np.load(cam_front_right_files_path)
           points3 = np.array(lidar_data['pcloud_points']) 
           
           cam_rear_center_files_path = os.path.join(cam_rear_center, cam_rear_center_files[i]) 
           lidar_data = np.load(cam_rear_center_files_path)
           points4 = np.array(lidar_data['pcloud_points']) 
           
           cam_side_left_files_path = os.path.join(cam_side_left, cam_side_left_files[i]) 
           lidar_data = np.load(cam_side_left_files_path) 
           points5 = np.array(lidar_data['pcloud_points']) 
           
           cam_side_right_files_path = os.path.join(cam_side_right, cam_side_right_files[i])
           lidar_data = np.load(cam_side_right_files_path)
           points6 = np.array(lidar_data['pcloud_points'])
           
           points = np.vstack((points1, points2, points3, points4, points5, points6)) 
           
           
           
           
      
           pcd = o3d.geometry.PointCloud()
           pcd.points = o3d.utility.Vector3dVector(points)
        
           obj.update_renderer(pcd)
           time.sleep(0.08) 
           
        
          
           
        

