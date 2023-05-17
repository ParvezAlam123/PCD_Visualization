import numpy as np
import rospy 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d 



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
            
            

obj = NonBlockVisualizer() 




def callback(data):
   points_list = []
   for point in pc2.read_points(data, skip_nans=True):
      points_list.append([point[0], point[1], point[2]])
         
   points = np.array(points_list)

   
   # filtered point cloud within specific range and visualize
   
   x = points[:, 0]
   y = points[:, 1]
   z = points[:, 2]
   
   mask = (y <= 0.75) & (y >= -0.75) 
   
   points = points[mask]
   
   pcd = o3d.geometry.PointCloud()
   pcd.points = o3d.utility.Vector3dVector(points)
   
   obj.update_renderer(pcd)
   
   
   
   
   
      
  
   
   
   
   
   
rospy.init_node("listener", anonymous=True)

rospy.Subscriber("velodyne_points", PointCloud2, callback)

rospy.spin()


