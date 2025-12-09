# parameters.py

# Screen and world settings
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
FPS = 30

# Wheel rotation limits (rad/s)
MAX_WHEEL_ROT_SPEED_RAD = 10.0
MIN_WHEEL_ROT_SPEED_RAD = -10.0

# Colors (RGB tuples)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


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
FOLLOWER_MAX_SPEED = 40.0
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
