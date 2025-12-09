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

class DifferentialDriveBot:
    def __init__(self, init_position, init_velocity, init_angle, max_speed, max_acceleration, max_angular_rate, dt=0.1, kp_pos = 0.1, kp_angular = 0.1, kd_pos = 0.01, kd_angular = 0.01):
        self.position = np.array(init_position)
        self.velocity = np.array(init_velocity)
        self.angle = init_angle
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_angular_rate = max_angular_rate
        self.dt = dt
        self.previous_error = None # vector indicating error
        self.previous_pos_error = None
        self.previous_angle_error = None
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        # self.ki_pos = ki_pos
        self.kp_angular = kp_angular
        self.kd_angular = kd_angular
        # self.ki_angular = ki_angular

    def update(self, acceleration_input, angular_velocity):
        if acceleration_input > self.max_acceleration:
            acceleration_input = self.max_acceleration
        if acceleration_input < -self.max_acceleration:
            acceleration_input = -self.max_acceleration

        if angular_velocity > self.max_angular_rate:
            angular_velocity = self.max_angular_rate
        if angular_velocity < -self.max_angular_rate:
            angular_velocity = -self.max_angular_rate


        self.velocity += acceleration_input * self.dt
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = self.velocity / speed * self.max_speed

        self.position += acceleration_input*np.array([math.cos(self.angle), math.sin(self.angle)])
        self.angle += angular_velocity * self.dt

    def get_fov_triangle(self, fov_angle=60, fov_length=150):
        half_fov = np.radians(fov_angle / 2)
        left_vertex = self.position + fov_length * np.array([np.cos(self.angle + half_fov), np.sin(self.angle + half_fov)])
        right_vertex = self.position + fov_length * np.array([np.cos(self.angle - half_fov), np.sin(self.angle - half_fov)])
        return [self.position, left_vertex, right_vertex]