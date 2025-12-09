# Leader-Follower Differential Drive Robot Sim

## Overview

Simulation of a leader robot (which is just a simple dot) which is followed by a differential-drive follower robot that avoids the obstacles by planning its path using RRT path planning and following this path using PID control. The follower maintains formation by chasing a virtual target (halo point) behind the leader while avoiding obstacles, as following is not the same as intercepting; the follower has to maintain a certain distance with the leader, hence the actual target is the 

https://github.com/user-attachments/assets/111b3d2b-0a68-4b5c-a74e-9db734600838

**Visualization:**
- **Red circle + line:** Leader robot and heading
- **Blue arrow:** Follower robot and orientation  
- **Red path:** Leader's planned trajectory
- **Light blue lines:** Follower's RRT path to halo point
- **Magenta circle:** Virtual target (halo point)
- **Black polygons:** Obstacles

---

## System Description

**Key Components:**
- **Leader:** Follows pre-planned RRT path at constant speed; provides heading for virtual target
- **Follower:** Differential drive robot; tracks waypoints to halo point using PID control
- **World:** Rectangular world user-defined dimensions with user-defined number of random polygonal obstacles 
- **Planning:** RRT (Rapidly-Exploring Random Tree) 
- **Control:** PID for linear and angular velocity

---

## Installation & Running

**Dependencies:**
```bash
# Clone the repo 
git clone https://github.com/NaitikDobariya/Differential_Drive_Robo_Follow.git

# Change the directory
cd Differential_Drive_Robo_Follow

# Create the virtual environment
python3 -m venv venv

# Activate the environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the simulation
python3 leader_follower_sim.py
```
- Sometimes due to improper initialization of leader initial position, leader goal position, follower initial position, or path of the leader, ther simulation will not run. In that case enter `ctrl + c` and just re-run the `leader_follower_sim.py` until the simulation gets initialized properly. This is an issue and will be dealt with in the future.

---

## Kinematics & Control

**Leader Model:**
- Follows the waypoint path at a constant speed
- Updates heading from velocity direction
- Halo point computed as: P_target = leader_pos - SAFE_DISTANCE * leader_heading_direction

**Follower Model:**
- State: (x, y, θ) - position and orientation
- Control: (v, ω) - linear and angular velocity  
- **Non-holonomic constraint:** Can only move forward in heading direction

**Control Law:**
1. PID computes error to next waypoint: e_pos = ||P_target - P_follower|| 
2. PID for heading: e_ang = arctan(Delta y / Delta x) - theta 
3. Apply PID gains

---

## File Structure

```
.
├── parameters.py              # Configuration
├── utils.py                   # Helpers (world gen, collision, rendering)
├── Car.py                     # Differential drive robot class
├── RRT_planner.py             # RRT algorithm + KDTree
├── PID_path_follower.py       # PID control function
├── b.py                       # Main simulation (leader-follower)
├── main_sim.py                # Alternative: single robot + path planning
└── simulation.py              # Alternative: full environment simulator
```

---

## Algorithm Flow (Per Frame)

1. **Update Leader:** Move along planned path, compute heading
2. **Compute Halo Point:** Virtual target behind leader
   - Try direct line behind the leader (12 samples)
   - Fallback to radial search if blocked (8 radii × 16 angles)
3. **Replan Follower** (every 1 second):
   - Adaptive RRT: RRT path between the current position of the robot and the halo point, choose iterations based on distance
   - Direct line of sight planning if close and unobstructed
   - Keep the last feasible path if planning fails
4. **Update Follower:**
   - PID computes velocity errors
   - Car class integrates differential equations
   - Update (x, y, θ)
5. **Render:** Display robots, paths, obstacles
6. **Repeat**



---

## Final Remarks

This thing might seem like a toy example and maybe even useless, the former is true  but thr later is not. It is a toy example but can be used in future projects with slight spin-off. For example an autonomous car tracking another car, a quadruped dog following its leader, or a quadcopter tracking an object of interest and performming surverillance.

This project still needs some important improvements, they will be done and then used for some of the aforementioned tasks on actual hardware.

Suggestions are welcome.
