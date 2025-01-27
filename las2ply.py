import os 
import laspy
import open3d as o3d
import numpy as np



path="/media/parvez/Expansion/h.las" 



# Input and output file paths
input_las_file = path
chunk_path = "/home/parvez/chunk_root"
output_ply_file = "output_file.ply" 



# Function to process a chunk and save it to the PLY file
def process_and_save_chunk(points, colors, output_ply_file, append=False):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        colors = colors / 65535.0  # Normalize colors to [0, 1]
        point_cloud.colors = o3d.utility.Vector3dVector(colors)
    
    # Save the chunk to the PLY file
    o3d.io.write_point_cloud(output_ply_file, point_cloud)

# Read the LAS file in chunks
chunk_size = 10**6  # Number of points per chunk
append_mode = False  # Start with overwrite mode
chunk = 0 

with laspy.open(input_las_file) as las:
    for points_chunk in las.chunk_iterator(chunk_size):
        # Extract point coordinates
        points = np.vstack((points_chunk.x, points_chunk.y, points_chunk.z)).transpose()

        # Extract colors if available
        colors = None
        #if las.header.point_format.has_color:
        colors = np.vstack((points_chunk.red, points_chunk.green, points_chunk.blue)).transpose()
        
        output_ply_file = os.path.join(chunk_path, str(chunk)+".ply")

        # Process and save the current chunk
        process_and_save_chunk(points, colors, output_ply_file, append=append_mode)
        append_mode = True  # Switch to append mode after the first chunk

        print("Processed a chunk")
        chunk = chunk + 1 

print(f"Converted {input_las_file} to {output_ply_file}")
