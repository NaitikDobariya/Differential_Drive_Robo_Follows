from parameters import *
import numpy as np 

class Leader:
    def __init__(self, path_points, speed=LEADER_SPEED):
        raw = [np.array(p, dtype=float) for p in path_points]
        # if len(raw) >= 2:
        #     sm = chaikin_smooth(raw, iterations=LEADER_SMOOTH_ITERS)
        #     self.path = resample_path_uniform(sm, spacing=LEADER_RESAMPLE_SPACING)
        # else:
        #     self.path = raw
        self.path = raw
        self.speed = speed
        self.pos = np.array(self.path[0], dtype=float)
        self.idx = 0
        self.finished = False
        self.vel = np.array([0.0, 0.0])

    def update(self, dt):
        if self.finished or self.idx >= len(self.path)-1:
            self.vel = np.array([0.0, 0.0])
            self.finished = True
            return
        
        target = self.path[self.idx+1]
        vec = target - self.pos
        dist = np.linalg.norm(vec)
        
        if dist < 1e-2:
            self.idx += 1
            if self.idx >= len(self.path)-1: 
                self.finished = True
            return
        
        dir_vec = vec / dist
        step = min(self.speed * dt, dist)
        self.pos += dir_vec * step
        self.vel = dir_vec * self.speed

    def heading_unit(self):
        n = np.linalg.norm(self.vel)
        if n < 1e-6:
            if self.idx < len(self.path)-1:
                vec = self.path[self.idx+1] - self.pos
                d = np.linalg.norm(vec)
                if d > 1e-6: 
                    return vec / d
            return np.array([1.0, 0.0])
        return self.vel / n