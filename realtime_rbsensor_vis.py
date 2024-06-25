import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField 
import open3d as o3d
import time
import numpy as np
import os 
import matplotlib.pyplot as plt 
import struct 




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
        
        

              
            
            
obj = NonBlockVisualizer()





def callback(msg):        

    
    # Extract the point cloud data
    pc_data = list(pc2.read_points(msg, skip_nans=True))
    
    # Convert to a numpy array
    points_array = np.array(pc_data, dtype=np.float32) 
    
    
    
    #print(points_array.shape)
    
    points = points_array[:, 0:3] 
    #print(points)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
        
    obj.update_renderer(pcd)
    #time.sleep(0.05)
        

rospy.init_node('pointcloud_subscriber')
sub = rospy.Subscriber('/rslidar_points', PointCloud2, callback)

rospy.spin()
