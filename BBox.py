import open3d as o3d
import numpy as np

def roty(t):
    """
    Rotation about the y-axis.
    """
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def box_center_to_corner(box):
        
    translation = box[3:6]
    h, w, l = box[0], box[1], box[2]
    #if the angle value is in radian then use below mentioned conversion
    # rotation_y = box[6]
    # rotation = rotation_y * (180/math.pi)                             #rad to degree
    rotation = box[6]

    # Create a bounding box outline if x,y,z is center point then use defination bounding_box as mentioned below
    # bounding_box = np.array([
    #     [-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
    #     [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
    #     [-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2]])
    
    # Create a bounding box outline if x,y,z is rear center point then use defination bounding_box as mentioned below
    bounding_box = np.array([
                [l,l,0,0,l,l,0,0],                      
                [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2],          
                [0,0,0,0,h,h,h,h]])                        

    # Standard 3x3 rotation matrix around the Z axis
    rotation_matrix = np.array([
        [np.cos(rotation), -np.sin(rotation), 0.0],
        [np.sin(rotation), np.cos(rotation), 0.0],
        [0.0, 0.0, 1.0]])

    # Standard 3x3 rotation matrix around the X axis
    # rotation_matrix = np.array([
    #     [1.0, 0.0, 0.0],
    #     [0.0, np.sin(rotation), np.cos(rotation)],
    #     [0.0, 0.0, 1.0]])

    #rotation_matrix = roty(rotation)

    # Repeat the [x, y, z] eight times
    eight_points = np.tile(translation, (8, 1))


    # Translate the rotated bounding box by the
    # original center position to obtain the final box
    corner_box = np.dot(
        rotation_matrix, bounding_box) + eight_points.transpose()
    print(corner_box.transpose())

    return corner_box.transpose()


#box = [h,w,l,x,y,z,rot]
box = [1.7,2.1,5.1,7.5,-0.1,-0.1,0.02]



pcd1 = o3d.io.read_point_cloud('/home/parvez/Dataset/PCD_data/1661166326.622939110.pcd')
points_v = np.asarray(pcd1.points)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0,0,0])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_v)
entities_to_draw = [pcd, mesh_frame]

boxes3d_pts = box_center_to_corner(box)
boxes3d_pts = boxes3d_pts.T
boxes3d_pts = o3d.utility.Vector3dVector(boxes3d_pts.T)
box = o3d.geometry.OrientedBoundingBox.create_from_points(boxes3d_pts)
box.color = [1, 0, 0]           #Box color would be red box.color = [R,G,B]
entities_to_draw.append(box)

    # Draw
o3d.visualization.draw_geometries([*entities_to_draw],
                                         front=[-0.9945, 0.03873, 0.0970],
                                         lookat=[38.4120, 0.6139, 0.48500],
                                         up=[0.095457, -0.0421, 0.99453],
                                         zoom=0.33799
                                         )
