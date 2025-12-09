import numpy as np
from parameters import *


class Car:
    def __init__(self, x, y, kp_pos = 0.1, kd_pos = 0.01, kp_angular = 0.01, kd_angular = 0.01):
           self.x = np.array([
                                [x],
                                [y],
                                [0]
                            ])

           self.x_dot = np.array([
                                [0],
                                [0],
                                [0]
                            ])

           self.wheel_speed = np.array([
                                            [0],
                                            [0]
                                        ])

           self.b = 25
           self.r = 5

           self.car_dims = np.array([
                                            [-self.b, -self.b, 1],
                                            [0 		, -self.b, 1],
                                            [ self.b,  		0, 1],
                                            [ 0, 	   self.b, 1],
                                            [ -self.b, self.b, 1]
                                        ])

           self.get_transformed_pts()
           self.position = self.x[0:2].flatten()
           self.angle = self.x[2][0]
           self.previous_error = None # vector indicating error
           self.previous_pos_error = None
           self.previous_angle_error = None
           self.kp_pos = kp_pos
           self.kd_pos = kd_pos
           self.kp_angular = kp_angular
           self.kd_angular = kd_angular



    def set_wheel_velocity(self, lw_speed, rw_speed):
            self.wheel_speed = np.array([
                                            [rw_speed],
                                            [lw_speed]
                                        ])
            self.x_dot = self.forward_kinematics()

    def set_robot_velocity(self, linear_velocity, angular_velocity):
            self.x_dot = np.array([
                                            [linear_velocity],
                                            [0],
                                            [angular_velocity]
                                        ])
            self.wheel_speed = self.inverse_kinematics()


    def update_state(self, dt):
            A = np.array([
                            [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]
                        ])
            B = np.array([
                            [np.cos(self.x[2, 0])*dt,  0],
                            [np.sin(self.x[2, 0])*dt,  0],
                            [0					 , dt]
                        ])

            vel = np.array([
                                [self.x_dot[0, 0]],
                                [self.x_dot[2, 0]]
                            ])
            self.x = A@self.x + B@vel


    def update(self, dt):
            self.wheel_speed[self.wheel_speed>MAX_WHEEL_ROT_SPEED_RAD] = MAX_WHEEL_ROT_SPEED_RAD
            self.wheel_speed[self.wheel_speed<MIN_WHEEL_ROT_SPEED_RAD] = MIN_WHEEL_ROT_SPEED_RAD
            self.x_dot = self.forward_kinematics()
            self.update_state(dt)
            self.wheel_speed = self.inverse_kinematics()
            self.position = self.x[0:2].flatten()
            self.angle = self.x[2,0]


    def get_state(self):
            return self.x, self.x_dot

    def forward_kinematics(self):
            kine_mat = np.array([
                                [self.r/2  		  , self.r/2],
                                [0 		 		  ,	0],
                                [self.r/(2*self.b), -self.r/(2*self.b)]
                                ])

            return kine_mat@self.wheel_speed

    def inverse_kinematics(self):
            ikine_mat = np.array([
                                [1/self.r, 0, self.b/self.r],
                                [1/self.r, 0, -self.b/self.r]
                                ])

            return ikine_mat@self.x_dot

    def get_transformed_pts(self):
            rot_mat = np.array([
                                [ np.cos(self.x[2, 0]), np.sin(self.x[2, 0]), self.x[0, 0]],
                                [-np.sin(self.x[2, 0]), np.cos(self.x[2, 0]), self.x[1, 0]],
                                [0, 0, 1]
                                ])

            self.car_points = self.car_dims@rot_mat.T

            self.car_points = self.car_points.astype("int")

    def get_points(self):
            self.get_transformed_pts()
            return self.car_points        
    
    def get_fov_triangle(self, fov_angle=60, fov_length=150):
        half_fov = np.radians(fov_angle / 2)
        left_vertex = self.position + fov_length * np.array([np.cos(self.angle + half_fov), np.sin(self.angle + half_fov)])
        right_vertex = self.position + fov_length * np.array([np.cos(self.angle - half_fov), np.sin(self.angle - half_fov)])
        return [self.position, left_vertex, right_vertex]