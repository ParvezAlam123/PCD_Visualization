import os 
import json 
import pandas as pd
import csv
# python imports
import math
import os, sys, argparse
import inspect
import json
import pdb
import numpy as np
from plyfile import PlyData, PlyElement
import open3d as o3d 


def represents_int(s):
    ''' if string s represents an int. '''
    try: 
        int(s)
        return True
    except ValueError:
        return False
 
 
def read_label_mapping(filename, label_from='raw_category', label_to='nyu40id'):
    assert os.path.isfile(filename)
    mapping = dict()
    with open(filename) as csvfile:
        reader = csv.DictReader(csvfile, delimiter='\t')
        for row in reader:
            mapping[row[label_from]] = int(row[label_to])
    if represents_int(list(mapping.keys())[0]):
        mapping = {int(k):v for k,v in mapping.items()}
    return mapping


def read_aggregation(filename):
    object_id_to_segs = {}
    label_to_segs = {}
    with open(filename) as f:
        data = json.load(f)
        num_objects = len(data['segGroups'])
        for i in range(num_objects):
            object_id = data['segGroups'][i]['objectId'] + 1 # instance ids should be 1-indexed
            label = data['segGroups'][i]['label']
            segs = data['segGroups'][i]['segments']
            object_id_to_segs[object_id] = segs
            if label in label_to_segs:
                label_to_segs[label].extend(segs)
            else:
                label_to_segs[label] = segs
    return object_id_to_segs, label_to_segs 
    
    
def read_segmentation(filename):
    seg_to_verts = {}
    with open(filename) as f:
        data = json.load(f)
        num_verts = len(data['segIndices'])
        #print(data['segIndices'])
        for i in range(num_verts):
            seg_id = data['segIndices'][i]
            if seg_id in seg_to_verts:
                seg_to_verts[seg_id].append(i)
            else:
                seg_to_verts[seg_id] = [i]
    return seg_to_verts, num_verts

def normalize_v3(arr):
    ''' Normalize a numpy array of 3 component vectors shape=(n,3) '''
    lens = np.sqrt( arr[:,0]**2 + arr[:,1]**2 + arr[:,2]**2 )
    arr[:,0] /= (lens + 1e-8)
    arr[:,1] /= (lens + 1e-8)
    arr[:,2] /= (lens + 1e-8)                
    return arr 
    
    
    
def compute_normal(vertices, faces):
    #Create a zeroed array with the same type and shape as our vertices i.e., per vertex normal
    normals = np.zeros( vertices.shape, dtype=vertices.dtype )
    #Create an indexed view into the vertex array using the array of three indices for triangles
    tris = vertices[faces]
    #Calculate the normal for all the triangles, by taking the cross product of the vectors v1-v0, and v2-v0 in each triangle             
    n = np.cross( tris[::,1 ] - tris[::,0]  , tris[::,2 ] - tris[::,0] )
    # n is now an array of normals per triangle. The length of each normal is dependent the vertices, 
    # we need to normalize these, so that our next step weights each normal equally.
    normalize_v3(n)
    # now we have a normalized array of normals, one per triangle, i.e., per triangle normals.
    # But instead of one per triangle (i.e., flat shading), we add to each vertex in that triangle, 
    # the triangles' normal. Multiple triangles would then contribute to every vertex, so we need to normalize again afterwards.
    # The cool part, we can actually add the normals through an indexed view of our (zeroed) per vertex normal array
    normals[ faces[:,0] ] += n
    normals[ faces[:,1] ] += n
    normals[ faces[:,2] ] += n
    normalize_v3(normals)
    
    return normals 
    
    
    
    
def export(mesh_file, agg_file, seg_file, meta_file, label_map_file, output_file=None):
    """ points are XYZ RGB (RGB in 0-255),
    semantic label as nyu40 ids,
    instance label as 1-#instance,
    box as (cx,cy,cz,dx,dy,dz,semantic_label)
    """
    label_map = read_label_mapping(label_map_file, label_from='raw_category', label_to='nyu40id')  
   
    pcd = o3d.io.read_point_cloud(mesh_file)
    mesh_vertices = np.asarray(pcd.points)

    # Load scene axis alignment matrix
    lines = open(meta_file).readlines()
    axis_align_matrix = None
    for line in lines:
        if 'axisAlignment' in line:
            axis_align_matrix = [float(x) for x in line.rstrip().strip('axisAlignment = ').split(' ')]

    if axis_align_matrix != None:
        axis_align_matrix = np.array(axis_align_matrix).reshape((4,4))
        pts = np.ones((mesh_vertices.shape[0], 4))
        pts[:,0:3] = mesh_vertices[:,0:3]
        pts = np.dot(pts, axis_align_matrix.transpose()) # Nx4
        aligned_vertices = np.copy(mesh_vertices)
        aligned_vertices[:,0:3] = pts[:,0:3]
    else:
        print("No axis alignment matrix found")
        aligned_vertices = mesh_vertices

    # Load semantic and instance labels
    if os.path.isfile(agg_file):
        object_id_to_segs, label_to_segs = read_aggregation(agg_file)
        seg_to_verts, num_verts = read_segmentation(seg_file)
        print(mesh_vertices.shape, num_verts)
        label_ids = np.zeros(shape=(num_verts), dtype=np.uint32) # 0: unannotated
        object_id_to_label_id = {}
        for label, segs in label_to_segs.items():
            label_id = label_map[label]
            for seg in segs:
                verts = seg_to_verts[seg]
                label_ids[verts] = label_id
        instance_ids = np.zeros(shape=(num_verts), dtype=np.uint32) # 0: unannotated
        num_instances = len(np.unique(list(object_id_to_segs.keys())))
        for object_id, segs in object_id_to_segs.items():
            for seg in segs:
                verts = seg_to_verts[seg]
                instance_ids[verts] = object_id
                if object_id not in object_id_to_label_id:
                    object_id_to_label_id[object_id] = label_ids[verts][0]
        
        instance_bboxes = np.zeros((num_instances,8)) # also include object id
        aligned_instance_bboxes = np.zeros((num_instances,8)) # also include object id
        for obj_id in object_id_to_segs:
            label_id = object_id_to_label_id[obj_id]

            # bboxes in the original meshes
            obj_pc = mesh_vertices[instance_ids==obj_id, 0:3]
            if len(obj_pc) == 0: continue
            # Compute axis aligned box
            # An axis aligned bounding box is parameterized by
            # (cx,cy,cz) and (dx,dy,dz) and label id
            # where (cx,cy,cz) is the center point of the box,
            # dx is the x-axis length of the box.
            xmin = np.min(obj_pc[:,0])
            ymin = np.min(obj_pc[:,1])
            zmin = np.min(obj_pc[:,2])
            xmax = np.max(obj_pc[:,0])
            ymax = np.max(obj_pc[:,1])
            zmax = np.max(obj_pc[:,2])
            bbox = np.array([(xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2, xmax-xmin, ymax-ymin, zmax-zmin, label_id, obj_id-1]) # also include object id
            # NOTE: this assumes obj_id is in 1,2,3,.,,,.NUM_INSTANCES
            instance_bboxes[obj_id-1,:] = bbox 


    return mesh_vertices, aligned_vertices, label_ids, instance_ids, instance_bboxes, instance_bboxes         
    
    
    
    
mesh_file = "/media/parvez/One_Touch/ScanNet/scans/scene0000_00/scene0000_00_vh_clean_2.ply" 
agg_file = "/media/parvez/One_Touch/ScanNet/scans/scene0000_00/scene0000_00.aggregation.json"
seg_file = "/media/parvez/One_Touch/ScanNet/scans/scene0000_00/scene0000_00_vh_clean_2.0.010000.segs.json" 
meta_file = "/media/parvez/One_Touch/ScanNet/scans/scene0000_00/scene0000_00.txt" 
label_map_file = "/media/parvez/One_Touch/ScanNet/scannetv2-labels.combined.tsv" 
output_file = None
 
 
mesh_vertices, aligned_vertices, label_ids, instance_ids, instance_bboxes, instance_bboxes = export(mesh_file, agg_file, seg_file, meta_file, label_map_file, output_file) 




pcd = o3d.io.read_point_cloud(mesh_file) 


# Example of bounding boxes, format: [cx, cy, cz, dx, dy, dz, category label, instance label]
bounding_boxes = instance_bboxes
# Create a list to hold all geometries
geometries = [pcd]

colors = []
m=0
n=0
o=0

for j in range(100):
    if j >=0 and j <20:
        colors.append([m+0.05, 0, 0])
        m = m + 0.05
    if j > 20 and j < 60:
        colors.append([m, j+0.05, 0])
        n = n+0.05 
    if j > 60:
        colors.append([m, n, o+0.05])
        o = o+0.05 

colors = np.array(colors)


# Loop through each bounding box
for i in range(bounding_boxes.shape[0]):
    box = bounding_boxes[i]
    cx, cy, cz, dx, dy, dz, category_label, instance_label = box

    # Create an OrientedBoundingBox centered at (cx, cy, cz) with extents (dx, dy, dz)
    bbox = o3d.geometry.OrientedBoundingBox(center=[cx, cy, cz],
                                             R=np.eye(3),  # Identity rotation (no rotation)
                                             extent=[dx, dy, dz])

    # Optionally set color based on the category or instance label
    bbox.color = colors[i] # Red color for all bounding boxes, change this as needed

    # Add the bounding box to the geometries list
    geometries.append(bbox)

# Visualize the point cloud and bounding boxes together with custom view parameters
o3d.visualization.draw_geometries(geometries)


