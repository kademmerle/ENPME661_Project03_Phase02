#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################
# Dependencies
#############################
import pygame
import pygame.gfxdraw
import time
import math
import heapq
import csv
import numpy as np

###########################
# Global Variables
###########################

# Screen dimensions
rows, cols = (540,300)

# Define Lists
OL = []
CL = {}
index_ctr = 0
solution = []
thresh = 0.5
clearance = 5
parent = {}
costsum = {}
theta_bins = 72
V = np.zeros((int(rows/thresh), int(cols/thresh), theta_bins))
C2C = np.zeros((int(rows/thresh), int(cols/thresh)))

# Define colors
pallet = {"white":(255, 255, 255), 
          "black":(0, 0, 0), 
          "green":(4,217,13), 
          "blue":(61,119,245), 
          "red":(242,35,24)
          }


# Check if x_prime is in the Open List
# Pops all items from OL, checks each to see if coordinates have already
# been visited, appends popped item to temp_list, then puts all items 
# back into OL
# Returns: True is present, False if not
def CheckOpenList(coords, open_list):
    temp_list = []
    present = False
    
    while open_list:
        item = heapq.heappop(open_list)
        if item[1] == coords:
            present = True
        heapq.heappush(temp_list, item)
        
    return present, temp_list
# END CheckOpenList()

"""
Check if x_prime is in the Closed List
Closed list is a dictionary with each node's coords used as a key
If x_prime coordinates are not in the closed list, a KeyError is thrown, which is caught by the try-except block.
Returns: True if present, False if not
"""
def CheckClosedList(coords, closed_list):
    
    try:
        if(closed_list[coords]):
            return True
    except KeyError:
        return False
# END CheckClosedList()

def round_and_get_v_index(node):
   #  Round x, y coordinates to nearest half to ensure we are on the center of a pixel
   
   x           = round(node[0] * 2, 1) / 2
   y           = round(node[1] * 2, 1) / 2
   theta_deg   = node[2]

   x_v_idx     = int(x * 2)
   y_v_idx     = int(y * 2)

   theta_deg_rounded = round(theta_deg / 5) * 5 # round to nearest 10 degrees
   theta_v_idx       = int(theta_deg_rounded % 360) // 5

   return (x, y, theta_deg_rounded), x_v_idx, y_v_idx, theta_v_idx
# END round_and_get_v_index()

"""
Utilizes function that was provided in Cost.py of the Proj 3 Phase 2 files. 
Variable names have been adjusted slightly, but general mechanics remain the same.
"""
def move_set(node, u_l, u_r):
    t = 0
    r = 3.3
    L = 28.7
    cost = 0
    dt = 0.1
    
    x_new = node[0]
    y_new = node[1]
    theta_new = 3.14 * node[2] / 180
    #print("Theta Start in rad: ", theta_new)
    
    while t < 1:
        t = t+dt
        x_new += (r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        theta_new += (r/L) * (u_r - u_l) * dt
        cost = cost + math.sqrt(math.pow(((r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        bcux = int(math.ceil(x_new))
        bcuy = int(math.ceil(y_new))
        if bcux <0 or bcuy < 0:
            return None
        bcdx = int(x_new)
        bcdy = int(y_new)
        if bcdx < 0 or bcdy < 0:
            return None
        if (bcux,bcuy) in buffer_set or (bcdx,bcdy) in buffer_set:
            #print("in buffer set")
            return None
    
    theta_new = int(180 * theta_new / 3.14)
    print("Estimated Cost to Come: ", cost)

    return (x_new, y_new, theta_new), round(cost, 1)
# END move_set()

def ValidMove(node):
    if((node[0] < 0) or (node[0] >= 540)):
        return False
    elif((node[1] < 0) or (node[1] >= 300)):
        return False
    else:
        return True
# END ValidMove()

def reverse_move(node,movement):
    t = 0
    r = 3.3
    L = 28.7
    cost = 0
    dt = -0.1
    x_new = node[0]
    y_new = node[1]
    u_l = movement[0]
    u_r = movement[1]
    theta_new = 3.14 * node[2] / 180
    #print("reverse Theta Start in rad: ", theta_new)
    xy_list = [(x_new,y_new)]
    
    while t > -1:
        t = t+dt
        theta_new += (r/L) * (u_r - u_l) * dt
        x_new += (r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        
        cost = cost + math.sqrt(math.pow(((r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        xy_list.append((round(x_new),round(y_new)))
        
    theta_new = int(180 * theta_new / 3.14)

    return xy_list
# END reverse_move()

# Define the object space for all letters/numbers in the maze
# Also used for determining if a set of (x,y) coordinates is present in 
# the action space
# Returns: True if in Object Space, False if not
def InObjectSpace(x, y):
        
        
    # Define Object 1
    if ((99<=x<=109) and (99<=y<=299)):
        return True
    
    # Define Object 2
    elif ((209<=x<=219) and (0<=y<=199)): 
        return True
    
    # Define Object 3
    elif (((319<=x<=329) and (0<=y<=99)) or \
          ((319<=x<=329) and (199<=y<=299))):
        return True
    
    # Define Object 4
    elif((429<=x<=439) and (99<=y<=299)):
        return True

    # Define Object Space for walls
    elif( ( (0<=x<=539) and y==0) or ((0<=x<=539) and y==299) ):
        return True

    # Default case, non-object space    
    else:
        return False
# END InObjectSpace()

# Backtrace the solution path from the goal state to the initial state
# Add all nodes to the "solution" queue
def GeneratePath(CurrentNode, parent, start_state):
    solution = []
    backtracing = True
    
    solution.append(CurrentNode)
    node = parent[CurrentNode]
    
    while backtracing:
        if(node[0] == start_state):
            solution.append(node)
            backtracing = False
        else:
            solution.append(node)
            node = parent[node]
            
    solution.reverse()
    return solution
# END GeneratePath()

"""
Calculate Euclidean Distance between current node and goal state

Euclidean Distance is the straight line distance between two points
Distance metric used in A* Search
"""
def euclidean_distance(node, goal_state):
    return math.sqrt((goal_state[0] - node[0])**2 + (goal_state[1] - node[1])**2)
# END euclidean_distance()

"""
 Draw the initial game board, colors depict:
  - White: In object space
  - Green: Buffer Zone
  - Black: Action Space
 Turn-clearance and robot radius used to calculate the total buffer zone that will be placed arround the walls and objects
 Robot will still be depicted on the screen as a "point-robot" as the center-point of the robot is the most import part for calculations

 Inputs:
  - rows:    x-axis size
  - cols:    y-axis size
  - pxarray: pixel array for screen that allows for drawing by point
  - pallet:  color dictionary with rgb codes
  - C2C:     (obsolete, not used here)
  - clear:   clearance needed for turns in mm
  - r:       robot raidus in mm

 Outputs: none
"""
def DrawBoard(rows, cols, pxarray, pallet, C2C, clear, r):
    buff_mod = clear + r
    for x in range(1,rows-1):
        for y in range(0,cols):
            in_obj = InObjectSpace(x,y)
            if (in_obj):
                pygame.draw.circle(screen,pygame.Color(pallet["green"]),(x,y),buff_mod,0)
  
            elif(0<y<=buff_mod or (298-buff_mod)<=y<299):
                    pxarray[x,y] = pygame.Color(pallet["green"])

    for x in range(0,rows):
        for y in range(0,cols):
            if pxarray[x,y] == 0:
                pxarray[x,y] = pygame.Color(pallet["white"])

    for x in range(0,rows):
        for y in range(0,cols):
            if InObjectSpace(x,y):
                pxarray[x,y] = pygame.Color(pallet["black"])
    
    for x in range(0,rows):
        for y in range(0,cols):
            if screen.get_at((x,y)) != pygame.Color(pallet["white"]):
                buffer_set.add((x,y))


def FillCostMatrix(C2C, pxarray, pallet, thresh):
    for x in range(0, int(rows/thresh)):
        for y in range(0, int(cols/thresh)):
            if((pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["black"])) or \
               (pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["green"]))):
                C2C[x,y] = -1
            else:
                C2C[x,y] = np.inf
#END FillCostMatrix()

#%%                    
def A_Star(start_node, goal_node, OL, parent, V, C2C, costsum, step, RPM1, RPM2):

    solution_path = []
    
    start_state, x_v_idx, y_v_idx, theta_v_idx = round_and_get_v_index(start_node[1])
    start_cost_state = (x_v_idx, y_v_idx, theta_v_idx)
    C2C[x_v_idx, y_v_idx] = 0.0
    costsum[start_cost_state] = 0.0 + euclidean_distance(start_node[1], goal_node)

    heapq.heappush(OL, start_node)
    
    while OL:
        node = heapq.heappop(OL)

        # Take popped node and center it, along with finding the index values for the V matrix
        fixed_node, x_v_idx, y_v_idx, theta_v_idx = round_and_get_v_index(node[1])
        
        # Add popped node to the closed list
        V[x_v_idx, y_v_idx, theta_v_idx] = 1
        arc_end = node[1]
        arc_speeds = node[2]
        arc_xy = reverse_move(arc_end,arc_speeds)
        #for i in range(0,len(arc_xy)):
        pygame.draw.lines(screen,pygame.Color(pallet["blue"]),False,arc_xy,1)
        arc_start = arc_xy[0]
        #pygame.draw.circle(screen, pygame.Color(pallet["red"]),arc_start,1,0)
            #pxarray[round(arc_x[i]),round(arc_y[i])] = pygame.Color(pallet["blue"])
        #pxarray[int(round(fixed_node[0])),int(round(fixed_node[1]))] = pygame.Color(pallet["blue"])
        pygame.display.update()
        
        # Check if popped node is within the goal tolerance region
        # If so, end exploration and backtrace to generate the soluion path
        # If not, apply action set to node and explore the children nodes
        if(euclidean_distance(fixed_node, goal_node) <= 1.5 ):
            solution_path = GeneratePath((fixed_node, arc_speeds), parent, start_state)
            return True, solution_path
        else:
            actions = [[0.0,  RPM1],
                       [RPM1,  0.0],
                       [RPM1, RPM1],
                       [0.0,  RPM2],
                       [RPM2,  0.0],
                       [RPM2, RPM2],
                       [RPM1, RPM2],
                       [RPM2, RPM1]]
            
            # Walk through each child node created by action set and determine if it has been visited or not
            for action in actions:
                #if move_set(fixed_node, action[0], action[1]) is not None:
                test = move_set(fixed_node, action[0], action[1])
                if test is not None:

                    child_node, child_cost = test
                    
                    if not ValidMove(child_node): continue
                    
                    child_node_fixed, child_x_v_idx, child_y_v_idx, child_theta_v_idx = round_and_get_v_index(child_node)
                    child_cost_node = (child_x_v_idx, child_y_v_idx, child_theta_v_idx)
                    
                    # Check if node is in obstacle space or buffer zone
                    try:
                        if((pxarray[int(child_node_fixed[0]), int(child_node_fixed[1])] == screen.map_rgb(pallet["black"])) or \
                           (pxarray[int(child_node_fixed[0]), int(child_node_fixed[1])] == screen.map_rgb(pallet["green"]))): continue
                    except IndexError:
                        continue  # Attempted move was outside bounds of the map
                    
                    # Check if node is in visited list
                    # If not, check if in open list using cost matrix C2C
                    if(V[child_cost_node] == 0):
                        
                        # If child is not in open list, create new child node
                        if(C2C[child_x_v_idx, child_y_v_idx]  == np.inf):
                            cost2go = round(euclidean_distance(child_node_fixed, goal_node),2)  # Calculate Cost to Go using heuristic equation Euclidean Distance
                            cost2come = C2C[x_v_idx, y_v_idx] + child_cost  # Calculate Cost to Come using parent Cost to Come and step size
                            parent[(child_node_fixed, (action[0],action[1]))] = (fixed_node, arc_speeds)     # Add child node to parent dictionary 
                            
                            C2C[child_x_v_idx, child_y_v_idx] = cost2come   # Update cost matrix with newly calculate Cost to Come
                            costsum[child_cost_node] = cost2come + cost2go  # Calculate the total cost sum and add to reference dictionary (this will be used when determiniing optimal path)
                            child = [costsum[child_cost_node], child_node_fixed,(action[0],action[1])]  # Create new child node --> [total cost, (x, y, theta)]... Total cost is used as priority determinant in heapq
                            heapq.heappush(OL, child)   # push child node to heapq
                        
                    # Child was in visited list, see if new path is most optimal
                    else:
                        cost2go = round(euclidean_distance(child_node_fixed, goal_node),2)
                        cost2come = C2C[x_v_idx, y_v_idx] + child_cost
                        
                        # Compare previously saved total cost estimate to newly calculated
                        # If new cost is lower than old cost, update in cost matrix and reassign parent 
                        if(costsum[child_cost_node] > (cost2come + cost2go)):  
                            parent[(child_node_fixed, (action[0],action[1]))] = (fixed_node, arc_speeds)
                            C2C[child_x_v_idx, child_y_v_idx] = cost2come
                            costsum[child_cost_node] = cost2come + cost2go
                            
    return False, solution_path
# END A_Star()

#%%
# Initialize pygame
pygame.init()

# Define screen size
window_size = (rows, cols)
screen = pygame.display.set_mode(window_size)
pxarray = pygame.PixelArray(screen)

#################################################
# Prompt User for starting and goal states
def GetUserInput():
    
    unanswered = True
    
    print("Welcome to the Point Robot Maze!")
    print("Before we get this show on the road, we need some information from you!\n")
    print("Robbie the Robot needs to navigate through a maze to get to get home...")
    print("Little Robbie is a bit forgetful though and needs your help remembering...")
    print("Where is he currently, and where is he going!\n")
    print("Robbie lives in a maze that is 180mm long and 50mm wide...")
    print("And he understands location using X and Y coordinates. So can you help?")
    print("---------------------------------------------------------------------------\n")
    while unanswered:
        start_x = float(input("Enter the starting x-coordinate: "))
        start_y = 299-float(input("Enter the starting y-coordinate: "))
        start_theta = float(input("Enter the starting orientation (0-360 degrees): "))
        
        # Check to see if start point falls in obstacle space, reprompt for new
        # coordinates if it does
        if(InObjectSpace(start_x, start_y)):
            print("Sorry, that start point appears to be object space!")
            continue
        
        goal_x = float(input("Enter the goal x-coordinate: "))
        goal_y = 299-float(input("Enter the goal y-coordinate: "))
        
        # Check to see if start point falls in obstacle space, reprompt for new
        # coordinates if it does
        if(InObjectSpace(goal_x, goal_y)):
            print("Sorry, that goal point appears to be in object space!")
            continue
        
        step_size = int(input("Enter step size from 1 to 10: "))
        if(step_size < 1 or step_size > 10):
            print("Sorry, that step size is not valid!")
            continue
        
        start_node = [0.0, (start_x, start_y, start_theta)]
        goal = (goal_x,goal_y)
     
        unanswered = False

    return start_node, goal, step_size
# END GetUserInput()

"""
 Collect input from user:
  - start_node: starting coordinates and orientation with total cost (float)
  - goal_node:  desired end coordinates as tuple (float)
  - step:       step size/movement length in mm (int)
  - rradius:    size of robot radius in mm (int)
"""
#start_node, goal_node, step, rradius = GetUserInput()

"""
 Draw board with objects and buffer zone
  - rows:      size x-axis (named at one point, and forgot to change)
  - cols:      size y-axis
  - pallet:    color library with RGB values for drawing on pixel array
  - C2C:       obsolete, starting costs set in FillCostMatrix()
  - clearance: turn clearance for robot in mm 
  - rradius:   robot radius in mm
"""
clearance = 0
start_node = [0.0, (10.0, 150.0, 0), (0,0)]
goal_node = (539.0, 85.0)
step = 0.1
RPM1 = 5.0
RPM2 = 10.0
r = 3.3
L = 22
buffer_set = set()

DrawBoard(rows, cols, pxarray, pallet, C2C, clearance, L)

# Update the screen
pygame.display.update()

# Grab start time before running the solver
start_time = time.perf_counter()

# Start running solver
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    FillCostMatrix(C2C, pxarray, pallet, thresh)
    
    # Start A_Star algorithm solver, returns game state of either SUCCESS (True) or FAILURE (false)
    alg_state, solution = A_Star(start_node, goal_node, OL, parent, V, C2C, costsum, step, RPM1, RPM2)
    
    if alg_state == False:
        print("Unable to find solution")
        end_time = time.perf_counter()
        running = False
    else:
        print("Solution found! Mapping now")
        end_time = time.perf_counter()
        running = False
    
    # Update the screen
    pygame.display.update()

# Calculate run time for solver
runtime = end_time - start_time

print("Time required to solve maze: ", runtime, " seconds")

# Draw start and goal points; start point will be filled, goal point will be hollow
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(start_node[1][0]), start_node[1][1]), radius=5.0, width=0) # Start node    
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(goal_node[0]), goal_node[1]), radius=5.0, width=1) # Goal node

# Draw solution path
final_path_xyt_list = []
final_path_drawing = []
for item in solution:
    xyt = (int(round(item[0][0])),int(round(item[0][1])),int(round(item[0][2])))
    final_curve = reverse_move(xyt,item[1])
    
    pygame.draw.lines(screen,pygame.Color(pallet["red"]),False,final_curve,2)
    pygame.display.update()


with open("src/turtlebot3_project3/scripts/waypoints.csv", "w", newline="") as file:
    writer = csv.writer(file)
    for item in solution:
        x = item[0]
        y = item[1]
        writer.writerow([x, y])
    
# Freeze screen on completed maze screen until user quits the game
# (press close X on pygame screen)
# running = True
# while running:
#     # handle events
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#             # quit pygame
#             pygame.quit()

# CONTROLLER

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import csv
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        #pkg_share = get_package_share_directory('turtlebot3_project3')
        #csv_path = os.path.join(pkg_share, 'scripts', 'waypoints.csv')
        csv_path = "src/turtlebot3_project3/scripts/waypoints.csv"
        self.waypoints = []


        # ---------- ROS Interfaces ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- Parameters ----------
        # List of waypoints (x, y), in meters
        r = 0.033 # wheel radius in meters

        with open(csv_path, 'r') as file:
            reader = csv.reader(file)
            w_l, w_r = next(reader)
            w_l = float(w_l) * 2 * math.pi / 60 # Convert to rad/s
            w_r = float(w_r) * 2 * math.pi / 60 # Convert to rad/s
            v_l = w_l*r
            v_r = w_r*r
            for row in reader:
                x_str, y_str = row
                x = float(x_str) / 100 # Convert from cm to m
                y = float(y_str) / 100 # Convert from cm to m
                y = y + (1.5-(2*y))
                self.waypoints.append((x, y))


        max_linear_velocity = (float(v_l) + float(v_r)) / 4 # (manual dampen)
        self.max_angular_velocity = max(
            ((float(w_r) - float(w_l)) / 2),
            ((float(w_l) - float(w_r)) / 2),
            float(w_l),
            float(w_r)
        ) / 2
        self.get_logger().info(f"Max linear velocity: {max_linear_velocity}")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_velocity}")

        self.current_waypoint_idx = 0
        self.distance_threshold = 0.05  # If within this move to the next waypoint
        
        # Base PID gains (we adapt Kp based on distance to path)
        self.base_kp = 1.0
        self.ki      = 0.1
        self.kd      = 0.1


        # PID state
        self.integral_error = 0.1
        self.prev_error     = 0.1

        # we'll just control angular.z to turn toward waypoint. For linear velocity, we use constant.
        self.forward_speed = max_linear_velocity  # m/s

        # Timer for control loop
        self.control_rate = 10.0  # Hz
        self.dt           = 1.0 / self.control_rate
        self.timer        = self.create_timer(self.dt, self.control_loop)

        # Robot pose from odometry
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.get_logger().info("Controller node started.")

    def odom_callback(self, msg):
        """
        Extract robot's (x, y, yaw) from odometry message
        """
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientation in quaternion form
        quat = msg.pose.pose.orientation
        # Convert quaternion to Euler yaw (assuming no pitch/roll)
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.yaw   = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """
        Periodically called function that:
        1) Pick current waypoint
        2) Compute heading error
        3) Adapt Kp based on distance to waypoint and apply modest integral and derivative gains
        4) Publish updated control command
        """

        # Check if we finished all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached. Stopping.")
            return

        # Current target waypoint
        wx, wy = self.waypoints[self.current_waypoint_idx]

        # Distance to current waypoint
        dx = wx - self.x
        dy = wy - self.y
        dist_error = math.hypot(dx, dy)

        # If close to the current waypoint, move on to the next
        if dist_error < self.distance_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}.")
            return

        # Compute heading to waypoint
        desired_yaw = math.atan2(dy, dx)

        # Heading error
        heading_error = self.normalize_angle(desired_yaw - self.yaw)

        # 3) Adapt Kp based on how far we are from the path
        gain_factor = 1.0 + min(dist_error, 1.0)  # cap at 1 meter for safety
        kp_adapted = self.base_kp * gain_factor

        # PID on heading error w/ adaptive P Gain
        self.integral_error += heading_error * self.dt
        deriv_error = (heading_error - self.prev_error) / self.dt

        control_angular = (kp_adapted * heading_error
                           + self.ki * self.integral_error
                           + self.kd * deriv_error)

        # Constuct and publish the command message
        cmd_msg = Twist()
        cmd_msg.linear.x = self.forward_speed

        # Limit angular velocity to max_angular_velocity
        if abs(control_angular) > abs(self.max_angular_velocity):
            control_angular = math.copysign(self.max_angular_velocity, control_angular)
        cmd_msg.angular.z = control_angular
        self.cmd_pub.publish(cmd_msg)

        # Store current error for next iteration
        self.prev_error = heading_error

        # Log
        self.get_logger().info(
            f"Waypoint: ({wx:.2f}, {wy:.2f}), Pos: ({self.x:.2f}, {self.y:.2f}), "
            f"DistErr: {dist_error:.2f}, HeadingErr: {heading_error:.2f}, "
            f"Kp: {kp_adapted:.2f}, AngZ: {control_angular:.2f}"
        )

    def stop_robot(self):
        """
        Publish zero velocity to stop the robot.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        """
        Normalize the angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
