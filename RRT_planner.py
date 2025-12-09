from IncrementalKDTree import IncrementalKDTree
from parameters import *
from numpy import random
import math
import numpy as np
from utils import *

def rrt_planner(start, end, max_iter, max_distance, end_threshold, SCREEN_WIDTH, SCREEN_HEIGHT, polygons):
    i = 0
    is_reached = False
    rrt_tree = IncrementalKDTree()
    rrt_tree.insert(start)
    
    while i < max_iter:
        if random.random() < 0.1:
            temp_sample = end  # Bias towards the goal
        else:
            temp_sample = [float(random.randint(0, SCREEN_WIDTH)), float(random.randint(0, SCREEN_HEIGHT))]

        temp_parent, temp_distance = rrt_tree.query(temp_sample, k=1)

        if not point_valid(temp_parent, temp_sample, polygons):
            continue

        # Handle the case where the sample is farther than max_distance
        if temp_distance > max_distance:
            line = LineString([temp_parent, temp_sample])
            perm_sample = line.interpolate(max_distance)
            perm_sample = (perm_sample.x, perm_sample.y)  # Ensure tuple format
        else:
            perm_sample = tuple(temp_sample)

        rrt_tree.insert(perm_sample, temp_parent)

        # Check distance to the goal
        distance = math.sqrt((end[0] - perm_sample[0])**2 + (end[1] - perm_sample[1])**2)
        if distance < end_threshold:
            rrt_tree.insert(end, perm_sample)  # Properly connect the end node
            print("Path found.")
            rrt_tree.path_found = True
            return rrt_tree
        
        i += 1
    
    print("No path found.")
    rrt_tree.path_found = False  # Indicate no path was found
    return rrt_tree