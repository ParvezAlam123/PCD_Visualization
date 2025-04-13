def vis_core(plys):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])  # Set background to black
    opt.point_size = 1.0

    PAR = os.path.dirname(os.path.abspath(__file__))
    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters(os.path.join(PAR, 'viewpoint.json'))
    for ply in plys:
        vis.add_geometry(ply)
    ctr.convert_from_pinhole_camera_parameters(param)

    vis.run()
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(os.path.join(PAR, 'viewpoint.json'), param)
    vis.destroy_window()











def vis_pc(pc, bboxes=None, labels=None):
    '''
    pc: ply or np.ndarray (N, 4)
    bboxes: np.ndarray, (n, 7) or (n, 8, 3)
    labels: (n, )
    '''
    #print("points = ", pc)
    if isinstance(pc, np.ndarray):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc[:, 0:3])
        #pc = npy2ply(pc)
        pc = pcd
    
    #mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    #size=10, origin=[0, 0, 0])
    
    #print("pc = ", pc)
    #print("bboxes = ", bboxes)
    if bboxes is None:
        vis_core([pc])
        return
    
    if len(bboxes.shape) == 2:
        bboxes = bbox3d2corners(bboxes)
    
    vis_objs = [pc]
    for i in range(len(bboxes)):
        bbox = bboxes[i]
        if labels is None:
            color = [1, 1, 0]
        else:
            if labels[i] >= 0 and labels[i] < 3:
                color = COLORS[labels[i]]
            else:
                color = COLORS[-1]
        vis_objs.append(bbox_obj(bbox, color=color))
    vis_core(vis_objs)

