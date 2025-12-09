# simulation_follow_leader_final2.py
import pygame
import numpy as np
import random
import math
import sys
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LineString, Point
from scipy.spatial import KDTree

# ----------------- Parameters -----------------
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
FPS = 60

# World / planner params
NUM_OBSTACLES = 30
OBSTACLE_INFLATION = 14.0
RRT_MAX_ITER_LEADER = 5000
RRT_MAX_DIST = 40.0
RRT_GOAL_THRESHOLD = 20.0

RRT_MAX_ITER_FOLLOWER = 1200
FOLLOWER_REPLAN_INTERVAL = 1.0  # seconds
SAFE_DISTANCE = 60.0  # desired safe following distance (pixels)

# Leader params
LEADER_SPEED = 10.0  # pixels/sec

# Follower params
FOLLOWER_MAX_SPEED = 80.0
FOLLOWER_MAX_OMEGA = 6.0  # rad/sec

# PID gains
LIN_KP = 1.0
LIN_KI = 0.0
LIN_KD = 0.05

ANG_KP = 3.0
ANG_KI = 0.0
ANG_KD = 0.08

# respawn tries if follower has to respawn
RESPAWN_ATTEMPTS = 50

# smoothing / display caps
LEADER_SMOOTH_ITERS = 3
LEADER_RESAMPLE_SPACING = 3.0
MAX_PATH_POINTS = 300

# ------------------------------------------------

# ---------- Utilities & Planner ----------
class IncrementalKDTree:
    def __init__(self):
        self.points = []


        
        self.parents = {}
        self.tree = None
        self.path_found = False

    def insert(self, new_point, parent_point=None):
        new_point = tuple(new_point)
        self.points.append(new_point)
        if parent_point is not None:
            self.parents[new_point] = tuple(parent_point)
        if self.points:
            self.tree = KDTree(self.points)

    def query(self, point, k=1):
        if not self.tree:
            return None, float('inf')
        distance, index = self.tree.query(point)
        return self.points[index], distance

    def get_final_path(self, start, end, max_points=MAX_PATH_POINTS):
        if not self.path_found:
            return []
        path = [tuple(end)]
        current = tuple(end)
        visited = set([current])
        safe_count = 0
        max_backtrace = max(1000, len(self.points) * 5)
        while tuple(current) != tuple(start) and safe_count < max_backtrace:
            parent = self.parents.get(tuple(current))
            if parent is None:
                print("[RRT] Missing parent during backtrace; returning partial path.")
                break
            if parent in visited:
                print("[RRT] Cycle detected in parent chain; aborting backtrace.")
                break
            path.append(parent)
            visited.add(parent)
            current = parent
            safe_count += 1
        if safe_count >= max_backtrace:
            print("[RRT] Backtrace exceeded safety limit; returning partial path.")
        path.reverse()
        if len(path) > max_points:
            indices = np.linspace(0, len(path)-1, max_points, dtype=int)
            path = [path[i] for i in indices]
        return [np.array(p, dtype=float) for p in path]


def generate_random_polygon(center, num_sides=5, radius=50):
    angle = 2 * math.pi / num_sides
    pts = []
    for i in range(num_sides):
        offset_angle = random.uniform(-angle/4, angle/4)
        theta = i * angle + offset_angle
        r = radius * random.uniform(0.7, 1.2)
        x = center[0] + r * math.cos(theta)
        y = center[1] + r * math.sin(theta)
        pts.append((x, y))
    return ShapelyPolygon(pts)


def create_world(num_polygons=NUM_OBSTACLES, plane_size=SCREEN_WIDTH, inflation_width=OBSTACLE_INFLATION):
    polygons = []
    inflated = []
    for _ in range(num_polygons):
        center = (random.uniform(0, plane_size), random.uniform(0, plane_size))
        num_sides = random.randint(3, 8)
        radius = random.uniform(18, 50)
        p = generate_random_polygon(center, num_sides, radius)
        polygons.append(p)
        inflated.append(p.buffer(inflation_width))
    return polygons, inflated


def point_free(pt, polygons):
    p = Point(tuple(pt))
    return not any(poly.contains(p) for poly in polygons)


def draw_polygon(polygon, surface, color):
    if isinstance(polygon, ShapelyPolygon):
        pts = [(int(x), int(y)) for x, y in polygon.exterior.coords]
        if len(pts) >= 3:
            pygame.draw.polygon(surface, color, pts)


def point_valid(parent, child, polygons):
    line = LineString([tuple(parent), tuple(child)])
    c = Point(tuple(child))
    for polygon in polygons:
        if polygon.contains(c) or line.intersects(polygon):
            return False
    return True


def rrt_planner(start, end, max_iter, max_distance, end_threshold, width, height, polygons):
    rrt = IncrementalKDTree()
    rrt.insert(tuple(start))
    for i in range(max_iter):
        sample = end if (random.random() < 0.12) else (float(random.randint(0, width)), float(random.randint(0, height)))
        nearest, dist = rrt.query(sample)
        if nearest is None:
            continue
        if not point_valid(nearest, sample, polygons):
            continue
        if dist > max_distance:
            line = LineString([nearest, sample])
            pt = line.interpolate(max_distance)
            new_point = (pt.x, pt.y)
        else:
            new_point = sample
        rrt.insert(new_point, nearest)
        if math.hypot(end[0] - new_point[0], end[1] - new_point[1]) < end_threshold:
            rrt.insert(end, new_point)
            rrt.path_found = True
            return rrt
    rrt.path_found = False
    return rrt


# ---------- Path smoothing ----------
def chaikin_smooth(points, iterations=2):
    pts = [tuple(p) for p in points]
    for _ in range(iterations):
        if len(pts) < 3:
            break
        new_pts = [pts[0]]
        for i in range(len(pts)-1):
            p0 = np.array(pts[i]); p1 = np.array(pts[i+1])
            q = 0.75*p0 + 0.25*p1
            r = 0.25*p0 + 0.75*p1
            new_pts.append(tuple(q)); new_pts.append(tuple(r))
        new_pts.append(pts[-1])
        pts = new_pts
    return [np.array(p, dtype=float) for p in pts]


def resample_path_uniform(points, spacing=4.0):
    if len(points) < 2:
        return [np.array(points[0], dtype=float)] if points else []
    pts = [np.array(p, dtype=float) for p in points]
    dists = [0.0]
    for i in range(1, len(pts)):
        dists.append(dists[-1] + np.linalg.norm(pts[i] - pts[i-1]))
    total_len = dists[-1]
    if total_len <= 1e-6:
        return [pts[0].copy()]
    num_samples = max(2, int(total_len / spacing) + 1)
    s_vals = np.linspace(0, total_len, num_samples)
    samples = []
    idx = 0
    for s in s_vals:
        while idx < len(dists)-2 and dists[idx+1] < s:
            idx += 1
        seg_len = dists[idx+1] - dists[idx]
        if seg_len == 0:
            samples.append(pts[idx].copy()); continue
        t = (s - dists[idx]) / seg_len
        p = (1-t)*pts[idx] + t*pts[idx+1]
        samples.append(p)
    return samples


# ----------------- PID -----------------
class PID:
    def __init__(self, kp, ki, kd, mn=-1e9, mx=1e9):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.mn = mn; self.mx = mx
        self.int_val = 0.0; self.last_error = None

    def reset(self):
        self.int_val = 0.0; self.last_error = None

    def update(self, error, dt):
        if dt <= 0: return 0.0
        self.int_val += error * dt
        deriv = 0.0
        if self.last_error is not None:
            deriv = (error - self.last_error) / dt
        self.last_error = error
        out = self.kp*error + self.ki*self.int_val + self.kd*deriv
        return max(self.mn, min(self.mx, out))


# ----------------- Leader -----------------
class Leader:
    def __init__(self, path_points, speed=LEADER_SPEED):
        raw = [np.array(p, dtype=float) for p in path_points]
        if len(raw) >= 2:
            sm = chaikin_smooth(raw, iterations=LEADER_SMOOTH_ITERS)
            self.path = resample_path_uniform(sm, spacing=LEADER_RESAMPLE_SPACING)
        else:
            self.path = raw
        self.speed = speed
        self.pos = np.array(self.path[0], dtype=float)
        self.idx = 0
        self.finished = False
        self.vel = np.array([0.0, 0.0])

    def update(self, dt):
        if self.finished or self.idx >= len(self.path)-1:
            self.vel = np.array([0.0, 0.0]); self.finished = True; return
        target = self.path[self.idx+1]
        vec = target - self.pos
        dist = np.linalg.norm(vec)
        if dist < 1e-2:
            self.idx += 1
            if self.idx >= len(self.path)-1: self.finished = True
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
                if d > 1e-6: return vec / d
            return np.array([1.0, 0.0])
        return self.vel / n


# ----------------- Follower -----------------
class Follower:
    def __init__(self, pose):
        self.x, self.y, self.theta = float(pose[0]), float(pose[1]), float(pose[2])
        self.v = 0.0; self.omega = 0.0
        self.lin_pid = PID(LIN_KP, LIN_KI, LIN_KD, mn=-FOLLOWER_MAX_SPEED, mx=FOLLOWER_MAX_SPEED)
        self.ang_pid = PID(ANG_KP, ANG_KI, ANG_KD, mn=-FOLLOWER_MAX_OMEGA, mx=FOLLOWER_MAX_OMEGA)
        self.path = []  # follower internal path used for motion
        self.path_index = 0

    def set_path(self, path_points):
        if not path_points:
            self.path = []; self.path_index = 0; return
        self.path = [np.array(p, dtype=float) for p in path_points]; self.path_index = 0
        self.lin_pid.reset(); self.ang_pid.reset()

    def current_pos(self):
        return np.array([self.x, self.y])

    def update(self, dt):
        # if no path, remain stopped
        if not self.path or self.path_index >= len(self.path):
            self.v = 0.0; self.omega = 0.0; return

        target = self.path[self.path_index]
        vec = target - np.array([self.x, self.y])
        dist = np.linalg.norm(vec)
        desired_heading = math.atan2(vec[1], vec[0])
        angle_err = (desired_heading - self.theta + math.pi) % (2*math.pi) - math.pi

        heading_penalty = max(0.0, 1.0 - abs(angle_err) / (math.pi/1.2))
        v_cmd = self.lin_pid.update(dist * heading_penalty, dt)
        v_cmd = max(-FOLLOWER_MAX_SPEED, min(FOLLOWER_MAX_SPEED, v_cmd))

        omega_cmd = self.ang_pid.update(angle_err, dt)
        omega_cmd = max(-FOLLOWER_MAX_OMEGA, min(FOLLOWER_MAX_OMEGA, omega_cmd))

        self.v = v_cmd; self.omega = omega_cmd

        # integrate
        self.theta += self.omega * dt
        self.theta = (self.theta + math.pi) % (2*math.pi) - math.pi
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        if dist < 8.0:
            self.path_index += 1

    def draw(self, surface):
        cx, cy = int(self.x), int(self.y); size = 10
        tri = [
            (cx + int(size * math.cos(self.theta)), cy + int(size * math.sin(self.theta))),
            (cx + int(size * math.cos(self.theta + 2.4)), cy + int(size * math.sin(self.theta + 2.4))),
            (cx + int(size * math.cos(self.theta - 2.4)), cy + int(size * math.sin(self.theta - 2.4)))
        ]
        pygame.draw.polygon(surface, (0,120,255), tri)
        pygame.draw.circle(surface, (0,0,0), (cx, cy), 3)

    def respawn_at(self, pt, theta=None):
        # teleport; ensure strictly free point passed in
        self.x = float(pt[0]); self.y = float(pt[1])
        self.theta = random.uniform(-math.pi, math.pi) if theta is None else float(theta)
        self.v = 0.0; self.omega = 0.0
        self.set_path([])
        print(f"[FOLLOWER] Respawned at {pt} theta {self.theta:.2f}")


# ----------------- Simulation loop -----------------
def simulate():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Leader-Follower")
    clock = pygame.time.Clock()

    polygons, inflated = create_world()
    background = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
    background.fill((255,255,255))
    for poly in polygons:
        draw_polygon(poly, background, (0,0,0))
    screen.blit(background, (0,0)); pygame.display.flip()

    def random_free_point(polys):
        for _ in range(2000):
            p = (float(random.randint(20, SCREEN_WIDTH-20)), float(random.randint(20, SCREEN_HEIGHT-20)))
            if point_free(p, polys):
                return p
        raise RuntimeError("No free spawn found")

    # plan leader until success
    while True:
        start_leader = random_free_point(polygons)
        goal_leader = random_free_point(polygons)
        print("[LEADER] Starting planning until success...")
        attempt = 0
        while True:
            attempt += 1
            rrt_tree = rrt_planner(start_leader, goal_leader, RRT_MAX_ITER_LEADER, RRT_MAX_DIST, RRT_GOAL_THRESHOLD, SCREEN_WIDTH, SCREEN_HEIGHT, inflated)
            if rrt_tree.path_found:
                print(f"[LEADER] Path found after {attempt} attempts.")
                break
            if attempt % 5 == 0:
                goal_leader = random_free_point(polygons)
                print(f"[LEADER] Rechoosing goal after {attempt} failures.")
        leader_path = rrt_tree.get_final_path(start_leader, goal_leader)
        if leader_path:
            break

    leader = Leader(leader_path, speed=LEADER_SPEED)
    print(f"[SIM] Leader path length (smoothed/resampled): {len(leader.path)}")

    # spawn follower sufficiently far
    follower_spawn = None
    for _ in range(200):
        pt = random_free_point(polygons)
        if np.linalg.norm(np.array(pt) - leader.pos) > SAFE_DISTANCE * 1.5:
            follower_spawn = pt; break
    if follower_spawn is None:
        follower_spawn = random_free_point(polygons)
    follower = Follower((follower_spawn[0], follower_spawn[1], random.uniform(-math.pi, math.pi)))

    # draw leader original path for context
    leader_path_lines = [(tuple(leader_path[i]), tuple(leader_path[i+1])) for i in range(len(leader_path)-1)]

    last_replan_time = -999.0
    running = True

    last_good_path = []
    consecutive_rrt_failures = 0

    print("[SIM] Entering main loop...")
    while running:
        dt = clock.tick(FPS) / 1000.0

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            if e.type == pygame.KEYDOWN and e.key == pygame.K_k:
                print("[DEBUG] Kill requested"); pygame.quit(); sys.exit(0)

        # update leader
        leader.update(dt)

        # compute virtual target behind leader
        leader_hat = leader.heading_unit()
        P_target = None
        for s in np.linspace(SAFE_DISTANCE, 0.0, 12):
            cand = leader.pos - s * leader_hat
            if point_free(cand, inflated):
                P_target = cand
                break
        if P_target is None:
            for r in np.linspace(SAFE_DISTANCE*0.2, SAFE_DISTANCE*1.2, 8):
                for ang in np.linspace(0, 2*math.pi, 16, endpoint=False):
                    cand = leader.pos + np.array([r*math.cos(ang), r*math.sin(ang)])
                    if 2 <= cand[0] <= SCREEN_WIDTH-2 and 2 <= cand[1] <= SCREEN_HEIGHT-2 and point_free(cand, inflated):
                        P_target = cand
                        break
                if P_target is not None:
                    break

        if P_target is not None:
            P_target = np.clip(P_target, [2,2], [SCREEN_WIDTH-2, SCREEN_HEIGHT-2])
            now = pygame.time.get_ticks() / 1000.0

            if now - last_replan_time >= FOLLOWER_REPLAN_INTERVAL:
                last_replan_time = now
                start_f = follower.current_pos()
                dist_to_target = np.linalg.norm(P_target - start_f)

                # ✅ Case 1: If follower is close → use direct line
                if dist_to_target < 120 and point_valid(start_f, tuple(P_target), inflated):
                    follower.set_path([tuple(start_f), tuple(P_target)])
                    last_good_path = follower.path.copy()
                    print("[FOLLOWER] Using direct LOS path (near leader).")
                else:
                    # ✅ Case 2: Medium distance → lightweight RRT
                    if dist_to_target < 200:
                        iter_budget = 350
                        local_max_dist = 40.0
                    else:
                        # ✅ Case 3: Far from leader → full RRT
                        iter_scale = 1 + int(dist_to_target // 150)
                        iter_budget = min(2000, int(RRT_MAX_ITER_FOLLOWER * iter_scale))
                        local_max_dist = min(120.0, RRT_MAX_DIST * (1 + dist_to_target / 300.0))

                    rrtf = rrt_planner(start_f, tuple(P_target), iter_budget, local_max_dist, 12.0, SCREEN_WIDTH, SCREEN_HEIGHT, inflated)

                    if rrtf.path_found:
                        new_path = rrtf.get_final_path(tuple(start_f), tuple(P_target), max_points=MAX_PATH_POINTS)
                        if new_path and len(new_path) >= 2:
                            follower.set_path(new_path)
                            last_good_path = new_path
                            consecutive_rrt_failures = 0
                            print(f"[FOLLOWER] Replanned SUCCESS; path_len={len(new_path)}")
                        else:
                            print("[FOLLOWER] Ignored RRT path (invalid).")
                    else:
                        print("[FOLLOWER ⚠️] RRT failed -> using last_good_path if available")
                        if last_good_path:
                            follower.set_path(last_good_path)

        # update follower motion
        follower.update(dt)

        # ---- Drawing ----
        screen.blit(background, (0,0))
        for a,b in leader_path_lines:
            pygame.draw.line(screen, (200,0,0), a, b, 2)

        # ✅ DRAW follower's current RRT path
        if last_good_path and len(last_good_path) > 1:
            for i in range(len(last_good_path)-1):
                pygame.draw.line(screen, (0,100,255),
                                 (int(last_good_path[i][0]), int(last_good_path[i][1])),
                                 (int(last_good_path[i+1][0]), int(last_good_path[i+1][1])), 2)

        # leader start/end & display
        pygame.draw.circle(screen, (0,255,0), (int(leader_path[0][0]), int(leader_path[0][1])), 6)
        pygame.draw.circle(screen, (0,0,255), (int(leader_path[-1][0]), int(leader_path[-1][1])), 6)

        pygame.draw.circle(screen, (255,0,0), (int(leader.pos[0]), int(leader.pos[1])), 6)
        head = leader.pos + leader.heading_unit() * 18.0
        pygame.draw.line(screen, (255,0,0), (int(leader.pos[0]), int(leader.pos[1])),
                         (int(head[0]), int(head[1])), 2)

        if P_target is not None:
            pygame.draw.circle(screen, (180,0,180),
                               (int(P_target[0]), int(P_target[1])), 6)

        follower.draw(screen)

        pygame.display.flip()

    pygame.quit()



if __name__ == "__main__":
    # prepare leader_path_lines variable (used in simulate scope)
    # We'll compute it inside simulate, but to avoid NameError keep placeholder
    leader_path_lines = []
    leader_path = []
    simulate()