import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
import argparse
import shutil
from std_srvs.srv import Empty
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

import sys
import moveit_commander
import numpy as np
from heapq import heappush, heappop
from math import sqrt
import random


import subprocess
import shlex

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import shutil


WIDTH, HEIGHT = 5.21, 2.75   # meters
RESOLUTION = 0.025            # meters per cell for generating the planning grid
NX = int(WIDTH/RESOLUTION) + 1
NY = int(HEIGHT/RESOLUTION) + 1

# inflate obstacles by this much since the turtlebot has roughly 0.5 m width
BUFFER = 0.0 # meters
MIN_CLEARANCE = 0.3  # meters


OBSTACLES = [
    (1.1, 1.65, 0.2, 0.4),
    (2.3, 1.65, 0.4, 0.4),
    (3.61, 1.65, 0.4, 0.2),
    (3.21, 0.7, 0.5, 0.2),
    (1.5, 0.6, 0.5, 0.2),
    (4.05, 0.0, 1.91, 0.2)
]


def build_grid():
    """Build occupancy grid with BUFFER inflation."""
    grid = np.zeros((NX, NY), dtype=np.uint8)
    for (cx, cy, w, h) in OBSTACLES:
        half_w = (w + 2*BUFFER) / 2
        half_h = (h + 2*BUFFER) / 2
        x0, x1 = cx - half_w, cx + half_w
        y0, y1 = cy - half_h, cy + half_h
        i0, i1 = max(0, int(x0/RESOLUTION)), min(NX-1, int(x1/RESOLUTION))
        j0, j1 = max(0, int(y0/RESOLUTION)), min(NY-1, int(y1/RESOLUTION))
        grid[i0:i1+1, j0:j1+1] = 1
    return grid

def world_to_grid(pt):
    """(x,y) in meters → (i,j) grid indices"""
    x, y = pt
    return (int(x/RESOLUTION), int(y/RESOLUTION))

def grid_to_world(ij):
    """(i,j) grid indices → (x,y) in meters at cell center"""
    i, j = ij
    return ((i + 0.5)*RESOLUTION, (j + 0.5)*RESOLUTION)

def create_safety_grid(obs_grid):
    """Mark cells within MIN_CLEARANCE of any obstacle as blocked."""
    safety = obs_grid.copy()
    margin = int(MIN_CLEARANCE/RESOLUTION)
    n,m = safety.shape
    for i in range(n):
        for j in range(m):
            if obs_grid[i,j]:
                for di in range(-margin, margin+1):
                    for dj in range(-margin, margin+1):
                        ni, nj = i+di, j+dj
                        if 0 <= ni < n and 0 <= nj < m:
                            safety[ni,nj] = 1
    return safety

def find_vertical_subgoal(safety_grid, cell):
    """If cell is blocked, pick a vertical displacement ±margin to a free cell."""
    margin = int(MIN_CLEARANCE/RESOLUTION)
    i, j = cell
    for sign in (+1, -1):
        jj = j + sign*margin
        if 0 <= jj < safety_grid.shape[1] and safety_grid[i, jj] == 0:
            return (i, jj)
    return cell

def astar_core(safety, start, goal):
    """Basic 8-connected A* on a binary safety grid."""
    moves = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
             (-1,-1,sqrt(2)),(-1,1,sqrt(2)),(1,-1,sqrt(2)),(1,1,sqrt(2))]
    def h(a,b):
        return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    open_set = []
    heappush(open_set, (h(start,goal), 0, start, None))
    came_from = {}
    gscore = {start:0}

    while open_set:
        f, g, node, parent = heappop(open_set)
        if node in came_from:
            continue
        came_from[node] = parent
        if node == goal:
            break
        for dx,dy,cost in moves:
            ni, nj = node[0]+dx, node[1]+dy
            if not (0 <= ni < safety.shape[0] and 0 <= nj < safety.shape[1]):
                continue
            if safety[ni,nj]:
                continue
            ng = g + cost
            nei = (ni,nj)
            if ng < gscore.get(nei, float('inf')):
                gscore[nei] = ng
                heappush(open_set, (ng + h(nei,goal), ng, nei, node))

    if goal not in came_from:
        return []
    # reconstruct
    path, cur = [], goal
    while cur:
        path.append(cur)
        cur = came_from[cur]
    return path[::-1]

def plan_segment(obs_grid, safety_grid, start, goal):
    """
    Plan from start→goal under safety_grid,
    inserting only goal‐side vertical subgoals if needed.
    """
    # need to make sure that when goal‐side is blocked we define a subgoal before moving forwards
    sub_goal = goal
    if safety_grid[goal]:
        sub_goal = find_vertical_subgoal(safety_grid, goal)

    path_safe = astar_core(safety_grid, start, sub_goal)
    if not path_safe:
        rospy.logwarn(f"ASTAR failed {start}→{sub_goal}")
        return []

    segment = path_safe[:]
    if sub_goal != goal:
        i,j = sub_goal
        ii,jj = goal
        step = 1 if jj>j else -1
        for dj in range(step, (jj-j)+step, step):
            segment.append((i, j+dj))

    return segment

def run_stp_planner(domain_pddl, problem_pddl):
    """
    Calls the STP planner and returns a list of action strings in execution order.
    """
    cmd = f"python2.7 /home/ttk4192/catkin_ws/src/temporal-planning-main/temporal-planning/bin/plan.py \
stp-2 {domain_pddl} {problem_pddl}"
#     cmd = f"python2.7 /root/catkin_ws/src/temporal-planning-main/temporal-planning/bin/plan.py \
# stp-2 {domain_pddl} {problem_pddl}"
#     # run planner
    # proc = subprocess.run(shlex.split(cmd), cwd="/root/catkin_ws/src/temporal-planning-main/temporal-planning",
    #                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    # proc = subprocess.run(shlex.split(cmd), cwd="/home/ttk4192/catkin_ws/src/temporal-planning-main/temporal-planning",
    #                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    proc = subprocess.run(shlex.split(cmd),
                      cwd="/home/ttk4192/catkin_ws/src/temporal-planning-main/temporal-planning",
                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    # print("Planner stdout:\n", proc.stdout)
    # print("Planner stderr:\n", proc.stderr)
    # the planner writes its solution into tmp_sas_plan.1 (or .2)
    plan_file = "/home/ttk4192/catkin_ws/src/temporal-planning-main/temporal-planning/tmp_sas_plan.1"
    
    actions = []
    with open(plan_file) as f:
        for line in f:
            line = line.strip()
            if line.startswith("(") or line.split(":",1)[0].replace('.','',1).isdigit():
                
                if "(" in line and ")" in line:
                    act = line[line.find("(")+1:line.find(")")]
                    actions.append(act)
    return actions

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = 0.0
        self.error = 0.0
        self.Derivator = 0
        self.Integrator = 0
        self.Integrator_max = 10
        self.Integrator_min = -10

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:
            self.error -= 2*pi
        elif self.error < -pi:
            self.error += 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator += self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        return self.P_value + self.I_value + self.D_value

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D


class turtlebot_move:
    def __init__(self):
        rospy.loginfo("Starting Turtlebot3 GNC Controller")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

        self.pid_theta = PID(P=2.0, I=0.0, D=0.1)

        self.trajectory = []

        self.last_move_time = rospy.Time.now()
        self.last_position = (0.0, 0.0)

    def move_to(self, goal_x, goal_y):
        """ Move to a given x,y goal with stuck detection + recovery """
        reached = False
        goal_dist_threshold = 0.05

        stuck_timeout = 6.0    # tuned these a bit, need quite a lot of seconds before declaring stuck otherwise at fine controlled behaviors it just randomly thinks it is stuck
        min_movement = 0.02    # meters

        self.last_move_time = rospy.Time.now()
        self.last_position = (self.x, self.y)

        while not rospy.is_shutdown() and not reached:
            diff_x = goal_x - self.x
            diff_y = goal_y - self.y
            distance = sqrt(diff_x**2 + diff_y**2)
            angle_to_goal = atan2(diff_y, diff_x)

            self.pid_theta.setPoint(angle_to_goal)
            angle_error = self.pid_theta.update(self.theta)

            max_angular = 0.5
            max_linear = 0.1
            angle_error = max(-max_angular, min(max_angular, angle_error))

            if abs(angle_error) > 0.1:
                self.vel.linear.x = 0.0
                self.vel.angular.z = angle_error
            else:
                self.vel.linear.x = min(max_linear, distance)
                self.vel.angular.z = angle_error

            self.vel_pub.publish(self.vel)

            if distance < goal_dist_threshold:
                reached = True
                break

            #check if the robot is stuck, really usabkle in sim, not the best in physical tests
            now = rospy.Time.now()
            dt = (now - self.last_move_time).to_sec()

            dx = self.x - self.last_position[0]
            dy = self.y - self.last_position[1]
            moved = sqrt(dx*dx + dy*dy)

            if dt > stuck_timeout:
                if moved < min_movement:
                    rospy.logwarn("[RECOVERY] Robot is stuck! Attempting recovery...")
                    self.stop()
                    rospy.sleep(0.5)
                    self.reverse(distance=0.15, speed=0.05)
                    self.last_move_time = rospy.Time.now()
                    self.last_position = (self.x, self.y)
                else:
                    # Update last move check
                    self.last_move_time = rospy.Time.now()
                    self.last_position = (self.x, self.y)

            self.rate.sleep()

        self.stop()

    def stop(self):
        """ Immediately stop the robot """
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)

    def reverse(self, distance=0.3, speed=0.05):
        """ Simple reverse movement """
        duration = distance / speed
        self.vel.linear.x = -speed
        self.vel.angular.z = 0.0
        t_end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()

    def odom_callback(self, msg):
        """ Update robot state from odometry """
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        orientation_q = pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.theta = yaw


def take_picture_action():
    camera = TakePhoto()
    now = rospy.Time.now()
    filename = f"scripts/photo_{now.secs}.jpg"

    rospy.sleep(2) 
    camera.take_picture(filename)

def manipulate_open_manipulator_action(open_gripper=True):
    req = SetJointPositionRequest()
    req.joint_position.joint_name = ["gripper"]
    req.joint_position.position   = [0.01 if open_gripper else 0.0]
    req.max_velocity_scaling_factor     = 1.0
    req.max_acceleration_scaling_factor = 1.0
    try:
        resp = goal_tool_srv(req)
        rospy.loginfo(f"{'Opening' if open_gripper else 'Closing'} gripper: {resp}")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call goal_tool_control: %s" % e)


class TakePhoto:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False

        img_topic = "/camera/rgb/image_raw" #used for simulation only!
        self.image_sub = rospy.Subscriber(img_topic, Image, self.image_callback)
        rospy.sleep(1)

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except Exception as e:
            rospy.logerr(e)

    def take_picture(self, filename):
        if self.image_received:
            cv2.imwrite(filename, self.cv_image)
            rospy.loginfo(f"Saved image {filename}")
            return True
        else:
            rospy.logwarn("No image received yet.")
            return False



def sparsify_path_by_turning(path, angle_threshold_deg=10):
    """ 
    Keep points if they make the robot turn more than angle_threshold_deg.
    Else skip points to make path sparser.
    """
    if len(path) < 3:
        return path

    keep = [path[0]] 

    def compute_angle(p1, p2, p3):
        """Compute angle between 3 points in degrees."""
        v1 = np.array([p2[0]-p1[0], p2[1]-p1[1]], dtype=np.float64)
        v2 = np.array([p3[0]-p2[0], p3[1]-p2[1]], dtype=np.float64)

        if np.linalg.norm(v1)==0 or np.linalg.norm(v2)==0:
            return 0
        v1 /= np.linalg.norm(v1)
        v2 /= np.linalg.norm(v2)
        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(dot)
        return np.degrees(angle)

    for i in range(1, len(path)-1):
        p1 = path[i-1]
        p2 = path[i]
        p3 = path[i+1]
        angle = compute_angle(p1, p2, p3)
        if angle > angle_threshold_deg:
            keep.append(p2)

    keep.append(path[-1]) 
    return keep

def sparsify_path(path, step_size=5):
    return path[::step_size] + [path[-1]]

import matplotlib.pyplot as plt

def plot_path(grid, path, filename=f"/home/ttk4192/catkin_ws/src/assigment4_ttk4192/scripts/astar_path{int(time.time())}.png"):
    """
    Plots the occupancy grid and overlays the A* path.
    """
    if not path:
        rospy.logwarn("No path to plot.")
        return
    
    plt.figure(figsize=(6, 3.5))
    width = grid.shape[0] * 0.05 
    height = grid.shape[1] * 0.05
    plt.imshow(grid.T, origin='lower', cmap='Greys', extent=[0, width, 0, height])
    
    wx = [grid_to_world(p)[0] for p in path]
    wy = [grid_to_world(p)[1] for p in path]
    plt.plot(wx, wy, '-r', label='A* path')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('A* Path on Occupancy Grid')
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    print('path saved to ', filename )
    plt.close()

def plot_grid(grid):
    """
    Plots the occupancy grid and saves it to a file with a timestamped filename.
    """
    plt.figure(figsize=(6, 3.5))
    width = grid.shape[0] * 0.05 
    height = grid.shape[1] * 0.05
    plt.imshow(grid.T, origin='lower', cmap='Greys', extent=[0, width, 0, height])
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Occupancy Grid')
    plt.tight_layout()
    
    filename = f"scripts/grid_plotted_napr28.png"
    plt.savefig(filename, dpi=300)
    print('Grid saved to', filename)
    plt.close()

def move_direct(x_start, y_start, x_goal, y_goal):
    """
    Simple PID‐only movement: ignore obstacles, just drive straight to the goal.
    """
    rospy.loginfo(f"[MOVE_DIRECT] from ({x_start:.2f},{y_start:.2f}) to ({x_goal:.2f},{y_goal:.2f})")
    mover = turtlebot_move()
    mover.move_to(x_goal, y_goal)


def move_astar(x_start, y_start, x_goal, y_goal):
    rospy.loginfo(f"[MOVE_ASTAR] {x_start:.2f},{y_start:.2f} → {x_goal:.2f},{y_goal:.2f}")
    obs_grid    = build_grid()
    safety_grid = create_safety_grid(obs_grid)
    plot_grid(obs_grid)

    start_cell = world_to_grid((x_start, y_start))
    goal_cell  = world_to_grid((x_goal,  y_goal))

    if safety_grid[start_cell] and x_start!=0.22:
        rospy.logwarn("Start is inside blocked area. Reversing...")
        reverse_robot(distance=MIN_CLEARANCE, speed=0.1)

        rospy.sleep(0.5)  
        mover = turtlebot_move()
        old_start = start_cell
        y_start_opt1 = y_start - 0.2
        y_start_opt2 = y_start + 0.2
        start_cell_opt1 = world_to_grid((x_start, y_start_opt1))
        start_cell_opt2 = world_to_grid((x_start, y_start_opt2))

        if safety_grid[start_cell_opt1]:
            start_cell = start_cell_opt2
        elif safety_grid[start_cell_opt2]:
            start_cell = start_cell_opt1
        
        rospy.loginfo(f"New start_cell after reversing: {start_cell}, old start cell: {old_start}")

    cell_path = plan_segment(obs_grid, safety_grid, start_cell, goal_cell)
    plot_path(obs_grid, cell_path)

    if not cell_path:
        rospy.logwarn("Planning failed; falling back to direct PID")
        return move_direct(x_start, y_start, x_goal, y_goal)

    mover = turtlebot_move()
    cell_path = sparsify_path_by_turning(cell_path, angle_threshold_deg=12)
    for cell in cell_path:
        wx, wy = grid_to_world(cell)
        mover.move_to(wx, wy)
    mover.move_to(x_goal, y_goal)



def reverse_robot(distance=MIN_CLEARANCE, speed=0.1):
    """
    Reverse the TurtleBot by a given distance (meters) at a given speed (m/s).
    """
    rospy.loginfo(f"[REVERSE] backing up {distance:.2f} m at {speed:.2f} m/s")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    vel.linear.x = -abs(speed)    

    duration = abs(distance / speed)
    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > duration:
            break
        pub.publish(vel)
        rate.sleep()

    vel.linear.x = 0
    pub.publish(vel)
    rospy.sleep(0.5)
    rospy.loginfo("[REVERSE] done")


def move_robot_action(x_start, y_start, x_goal, y_goal, move_method='pid'):
    """
    Dispatch to the selected movement backend.
    """
    if move_method == 'astar':
        move_astar(x_start, y_start, x_goal, y_goal)
    elif move_method == 'pid':
        move_direct(x_start, y_start, x_goal, y_goal)
    else:
        rospy.logwarn(f"Unknown move_method '{move_method}', using PID")
        move_direct(x_start, y_start, x_goal, y_goal)

def init_manipulator():
    global manip_pub
    manip_pub = rospy.Publisher(
        '/open_manipulator/goal_tool_control',
        String, queue_size=10
    )
    # give the publisher a moment to register
    rospy.sleep(1)


def close_gripper():
    target = { 'gripper': 0.0 }
    gripper_group.set_joint_value_target(target)
    gripper_group.go(wait=True)
    gripper_group.stop()
    rospy.loginfo("Gripper closed.")

def open_gripper():
    target = { 'gripper': 0.01 }
    gripper_group.set_joint_value_target(target)
    gripper_group.go(wait=True)
    gripper_group.stop()
    rospy.loginfo("Gripper opened.")



def init_manipulator_service():
    rospy.wait_for_service('/open_manipulator/goal_tool_control')
    return rospy.ServiceProxy('/open_manipulator/goal_tool_control', SetJointPosition)

if __name__=='__main__':
    rospy.init_node('mission_planner', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    
    joint_names = gripper_group.get_active_joints()

    print((f"[DEBUG] active joints: {gripper_group.get_active_joints()}"))
    print((f"[DEBUG] planning-group joints: {gripper_group.get_joints()}"))
    rospy.loginfo(f"[DEBUG] gripper group joints: {joint_names}")

    
    current_vals = gripper_group.get_current_joint_values()
    rospy.loginfo(f"[DEBUG] current gripper values: {current_vals}")
    print('current vals',current_vals, joint_names)
    
    print('init mission planner done')
    print('waiting is over')
    domain="/mission-domain.pddl"
    problem="/mission-problem.pddl"
    plan_general = run_stp_planner(domain, problem)
    print("STP plan:", plan_general)
    USE_ASTAR = True

    waypoints = {
       "waypoint0": (0.45, 0.32),
       "waypoint1": (1.49, 0.35),
       "waypoint2": (3.41, 1.38),
       "waypoint3": (3.75, 2.25),
       "waypoint4": (4.60, 0.65),
       "waypoint5": (0.76, 2.25),
       "waypoint6": (3.61, 1.21)
     }
    
    #defining points around the map used for gathering data. These would have to be known in an unknown environment, but a simple exploration policy could be used to map data also
    discover = {
       "discover0": (0.45, 0.32),
       "discover1": (4.60, 0.45),
       "discover2": (3.75, 2.25),
       "discover3": (0.56, 2.25)
     }
    discovery_sequence = ["discover1", "discover2", "discover3", "discover0"]
    for i in range(len(discovery_sequence)-1):
        wp_from = discovery_sequence[i]
        wp_to = discovery_sequence[i+1]
        x0, y0 = discover[wp_from]
        x1, y1 = discover[wp_to]
        
        if USE_ASTAR:
            move_astar(x0, y0, x1, y1)
        else:
            move_direct(x0, y0, x1, y1)

    for act in plan_general:
        print("→", act)
        parts = act.split()
        op = parts[0]
        
        if op == "move":
            _, _, wp_from, wp_to, _ = parts
            x0, y0 = waypoints[wp_from]
            x1, y1 = waypoints[wp_to]
            
            if USE_ASTAR:
                move_astar(x0, y0, x1, y1)
            else:
                move_direct(x0, y0, x1, y1)

        elif op == "manipulate-valve":
            close_gripper()
            rospy.sleep(1.0)
            open_gripper()
            rospy.sleep(1.0)
        # elif op == "manipulate-valve":
        #     _, _, valve, wp = parts[0:4]
        #     manipulate_open_manipulator_action(open_gripper=False)
        
        elif op == "take-picture":
            _, _, pump, wp = parts[0:4]
            take_picture_action()
        
        else:
            rospy.logwarn(f"Unknown PDDL op {op}")
    
    rospy.sleep(1)

    rospy.loginfo("Mission complete.")
    rospy.spin()
