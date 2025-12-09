from scipy.spatial import KDTree
import numpy as np

class IncrementalKDTree:
    def __init__(self):
        # Initialize with an empty list or provided points
        self.points = []
        self.parents = {}
        # Build KDTree if there are any points
        self.tree = None
        self.path_found = False
    def insert(self, new_point, parent_point = None):
        # Append new point to the list of points
        new_point = tuple(new_point)
        self.points.append(new_point)
        # Rebuild KDTree with the new set of points
        if parent_point is not None:
            parent_point = tuple(parent_point)
            if parent_point not in self.points:
                raise Exception("Error") 
            
            self.parents[new_point] = parent_point  # Store the parent relationship

        self.tree = KDTree(self.points)
    
    def points_in_tree(self):
        return self.points
    
    def query(self, point, k=1):
        if len(self.points) == 1:
            # If only one point is in the tree, return it directly
            return self.points[0], np.linalg.norm(np.array(self.points[0]) - np.array(point))
        
        if self.tree:
            # Query the KDTree for the nearest neighbor
            distance, index = self.tree.query(point)
            nearest_point = self.points[index]
            return nearest_point, distance
        
        return None, float('inf')
    
    def get_final_path(self, start, end):
        if self.path_found:
            path = [end]
            current_point = tuple(end)
            # while current_point != tuple(start):
            while np.linalg.norm(np.array(current_point) - np.array(start)) > 2.0:
                current_point = self.parents[current_point]
                path.append(current_point)

            path.reverse()
            return path
        
        else:
            return []