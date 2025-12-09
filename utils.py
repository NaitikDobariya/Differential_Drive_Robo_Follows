import pygame
import numpy as np
import random
from shapely.geometry import Polygon as ShapelyPolygon
# from shapely.affinity import translate, rotate
# import math
from shapely.geometry import LineString, Point, MultiLineString
# from scipy.spatial import KDTree
from parameters import *
# import threading

def generate_random_polygon(center, num_sides=5, radius=50):
    angle = 2 * 3.141592653589793 / num_sides
    points = []
    for i in range(num_sides):
        offset_angle = random.uniform(-angle/4, angle/4)
        theta = i * angle + offset_angle
        x = center[0] + radius * random.uniform(0.8, 1.2) * np.cos(theta)
        y = center[1] + radius * random.uniform(0.8, 1.2) * np.sin(theta)
        points.append((x, y))
    return ShapelyPolygon(points)

def create_world(num_polygons=10, plane_size=SCREEN_WIDTH, inflation_width = 15):
    polygons = []
    inflated_polygons = []
    for _ in range(num_polygons):
        center = (random.uniform(0, plane_size), random.uniform(0, plane_size))
        num_sides = random.randint(3, 8)
        radius = random.uniform(20, 50)
        polygon = generate_random_polygon(center, num_sides, radius)
        polygons.append(polygon)

        inflated_polygon = polygon.buffer(inflation_width)
        inflated_polygons.append(inflated_polygon)

    return polygons, inflated_polygons

def draw_polygon(polygon, surface, color, scale_factor=(1, 1)):
    if isinstance(polygon, ShapelyPolygon):
        scaled_coords = [(int(x * scale_factor[0]), int(y * scale_factor[1])) for x, y in polygon.exterior.coords]
        pygame.draw.polygon(surface, color, scaled_coords)
    elif isinstance(polygon, list):  # Handles the case where the polygon is a list of points (FOV)
        scaled_coords = [(int(x * scale_factor[0]), int(y * scale_factor[1])) for x, y in polygon]
        pygame.draw.polygon(surface, color, scaled_coords)


def draw_arrow(screen, position, angle):
    # Define the arrow as a triangle with three points, this is the car
    arrow_length = 20
    arrow_width = 10
    point1 = tuple(position + arrow_length * np.array([np.cos(angle), np.sin(angle)]))
    point2 = tuple(position + arrow_width * np.array([np.cos(angle + np.pi / 2), np.sin(angle + np.pi / 2)]))
    point3 = tuple(position + arrow_width * np.array([np.cos(angle - np.pi / 2), np.sin(angle - np.pi / 2)]))
    pygame.draw.polygon(screen, RED, [point1, point2, point3])
    return ShapelyPolygon([point1, point2, point3])


def point_valid(parent, child, polygons): # returns whether a point randomly choosen is valid or not
    line = LineString([tuple(parent), tuple(child)])
    c = Point(tuple(child))
    for polygon in polygons:
        if polygon.contains(c) or line.intersects(polygon):
            return False
    
    return True

def random_free_point(polygons):
    for _ in range(2000):
        p = (
            float(random.randint(20, SCREEN_WIDTH - 20)),
            float(random.randint(20, SCREEN_HEIGHT - 20))
        )
        if point_free(p, polys):
            return p
    raise RuntimeError("No free spawn found")

def simulate_lidar(r, bot, polygons, fov_shape, fov_angle, main_screen = None):
	lidar_points = []
	for k in range(-int(r/2), int(r/2+1)):
	            angle = bot.angle + k * np.radians(fov_angle)/r
	            line_end = bot.position + 150 * np.array([np.cos(angle), np.sin(angle)])
	            line = LineString([tuple(bot.position), tuple(line_end)])

	            closest_intersection = None
	            min_distance = float('inf')

	            for polygon in polygons:
	                if fov_shape.intersects(polygon):
	                    intersecting_area = fov_shape.intersection(polygon)
	                    if not intersecting_area.is_empty:
	                        # Find intersection between the line and the intersecting area
	                        intersection = intersecting_area.intersection(line)
	                        if not intersection.is_empty:
	                            if isinstance(intersection, Point):
	                                distance = np.linalg.norm(np.array(intersection.coords[0]) - bot.position)
	                                if distance < min_distance:
	                                    closest_intersection = intersection
	                                    min_distance = distance
	                            elif isinstance(intersection, LineString):
	                                for point in intersection.coords:
	                                    point_geom = Point(point)
	                                    distance = np.linalg.norm(np.array(point) - bot.position)
	                                    if distance < min_distance:
	                                        closest_intersection = point_geom
	                                        min_distance = distance
	                            elif isinstance(intersection, MultiLineString):
	                                for linestring in intersection.geoms:  # Correct way to iterate over LineStrings in a MultiLineString
	                                    for point in linestring.coords:
	                                        point_geom = Point(point)
	                                        distance = np.linalg.norm(np.array(point) - bot.position)
	                                        if distance < min_distance:
	                                            closest_intersection = point_geom
	                                            min_distance = distance

	            # Transform the closest intersection point to the bot's frame
	            if closest_intersection:
	                lidar_points.append([closest_intersection.x, closest_intersection.y])
	                if main_screen:
	                    pygame.draw.circle(main_screen,RED, (closest_intersection.x, closest_intersection.y), 2)

	return lidar_points

def waypoint_generator(path, bot, current_index): # gives the index of waypoint that needs to be taken as goal
    if current_index == len(path)-1:
         return None
    if current_index == None:
         return -1
    if np.linalg.norm(bot.position - path[current_index]) < 15:
         return current_index+1
    else:
         return current_index

