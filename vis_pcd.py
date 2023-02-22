import open3d as o3d
import os
import numpy as np
import time
import matplotlib.pyplot as plt


# Define directory containing point cloud files
dir_path = "/home/parvez/Data/Dataset/training/SteeringData/"

os.environ["OPEN3D_VISUALIZER"] = "gl"

# Create Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()
                 
vis.get_render_option().point_size=1
vis.get_render_option().background_color = [0.0, 0.0, 0.0]
vis.get_render_option().show_coordinate_frame = True






# Loop over all files in directory
for filename in sorted(os.listdir(dir_path)):
    if filename.endswith(".pcd"):
    
        # Load point cloud file
        pcd = o3d.io.read_point_cloud(os.path.join(dir_path, filename))
        
        # Get the intensity values of the points
        intensity = np.asarray(pcd.points)[:, 2]
        

        # Normalize the intensity values to the range [0, 1]
        intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())

        # Convert the normalized intensity values to RGB colors
        colors = plt.cm.jet(intensity_normalized)[:, :3]

        # Set the colors of the point cloud
        pcd.colors = o3d.utility.Vector3dVector(colors)
                        
        # Add point cloud to visualizer
        vis.add_geometry(pcd)
        

        # Update visualizer
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        
        
        vis.clear_geometries()
        
       
        

    
vis.destroy_window()       
        
       
        

      
