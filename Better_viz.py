import numpy as np
import open3d as o3d
import time
import os

def play_motion(list_of_pcds: []):
    play_motion.vis = o3d.visualization.Visualizer()
    play_motion.index = 0

    def reset_motion(vis):
        play_motion.index = 0
        pcd.points = list_of_pcds[0].points
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(.25)
        vis.register_animation_callback(forward)
        return False

    def backward(vis):
        pm = play_motion

        if pm.index > 0:
            pm.index -= 1
            pcd.points = list_of_pcds[pm.index].points
            time.sleep(.05)
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
        else:
            vis.register_animation_callback(forward)

    def forward(vis):
        pm = play_motion
        if pm.index < len(list_of_pcds) - 1:
            pm.index += 1
            pcd.points = list_of_pcds[pm.index].points
            pcd.colors = list_of_pcds[pm.index].colors
            time.sleep(0.1)
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
            #time.sleep(0.005)
        #else:
        #    # vis.register_animation_callback(reset_motion)
        #    #vis.register_animation_callback(backward)
        return False

    # Geometry of the initial frame
    pcd = o3d.geometry.PointCloud()
    first = np.array(list_of_pcds[0].points)
    pcd.points = o3d.utility.Vector3dVector(first.reshape(-1, 3))
    pcd.colors = o3d.utility.Vector3dVector(np.ones(first.reshape(-1, 3).shape) * [0,0,0])

    # Initialize Visualizer and start animation callback
    vis = play_motion.vis
    vis.create_window(window_name='')
    ctr = vis.get_view_control()
    ctr.rotate(0, -50)
    vis.add_geometry(pcd)
    vis.register_animation_callback(forward)
    vis.run()
    vis.destroy_window()



input_dir = "/home/parvez/Dataset/PCD_data"
files = os.listdir(input_dir)

    
pcds = []
for i in range(len(files)):
    file_path = os.path.join(input_dir, files[i])
    pcd = o3d.io.read_point_cloud(file_path)
    #print (pcd.get_center())
    pcds.append(pcd)
play_motion(pcds)
