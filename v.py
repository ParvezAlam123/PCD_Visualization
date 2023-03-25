import open3d as o3d
import numpy as np

class BoundingBox:
    def __init__(self, center, size):
        self.center = center
        self.size = size
        
    def get_corners(self):
        half_size = self.size / 2
        corners = np.array([
            [-1, -1, -1],
            [ 1, -1, -1],
            [ 1,  1, -1],
            [-1,  1, -1],
            [-1, -1,  1],
            [ 1, -1,  1],
            [ 1,  1,  1],
            [-1,  1,  1]
        ]) * half_size
        corners += self.center
        return corners
    
class Scene:
    def __init__(self):
        self.bounding_boxes = []
        self.line_sets = []
        
    def add_bounding_box(self, center, size):
        self.bounding_boxes.append(BoundingBox(center, size))
        
    def update_line_sets(self):
        self.line_sets.clear()
        for box in self.bounding_boxes:
            corners = box.get_corners()
            lines = np.array([
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]
            ])
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(np.tile(np.array([1, 0, 0]), (len(lines), 1)))
            self.line_sets.append(line_set)
        
    def get_visualization(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for line_set in self.line_sets:
            vis.add_geometry(line_set)
        return vis

# create a scene with two bounding boxes
scene = Scene()
scene.add_bounding_box(np.array([1, 1, 1]), np.array([0.5, 1.0, 0.5]))
scene.add_bounding_box(np.array([-1, -1, -1]), np.array([0.5, 0.5, 1.0]))

# update the line sets to reflect the current state of the scene
scene.update_line_sets()

# visualize the scene
vis = scene.get_visualization()
vis.run()
vis.destroy_window()
