#!/usr/bin/env python3

# %%
# -*- coding: utf-8 -*-

#############################
# Dependencies
#############################
import pygame
import pygame.gfxdraw
import time
import math
import heapq
import numpy as np
import csv




def CheckOpenList(coords, open_list):
    '''
    Check if coords is in the Open List
    Pops all items from OL, checks each to see if coordinates have already
    been visited, appends popped item to temp_list, then puts all items 
    back into OL
    Returns: True is present, False if not
    '''
    temp_list = []
    present = False
    
    while open_list:
        item = heapq.heappop(open_list)
        if item[1] == coords:
            present = True
        heapq.heappush(temp_list, item)
        
    return present, temp_list


def CheckClosedList(coords, closed_list):
    """
    Check if x_prime is in the Closed List
    Closed list is a dictionary with each node's coords used as a key
    If x_prime coordinates are not in the closed list, a KeyError is thrown, 
    which is caught by the try-except block.
    Returns: True if present, False if not
    """
    
    try:
        if(closed_list[coords]):
            return True
    except KeyError:
        return False



def round_and_get_v_index(node):
   #  Round x, y coordinates to nearest half to ensure our indexing is aligned 
   #  Round Theta to nearest 5 degrees
   
   x           = round(node[0] * 2, 1) / 2
   y           = round(node[1] * 2, 1) / 2
   theta_deg   = node[2]

   x_v_idx     = int(x * 2)
   y_v_idx     = int(y * 2)

   theta_deg_rounded = round(theta_deg / 5) * 5 # round to nearest 5 degrees
   theta_v_idx       = int(theta_deg_rounded % 360) // 5

   return (x, y, theta_deg_rounded), x_v_idx, y_v_idx, theta_v_idx

def move_set(node, u_l, u_r, buffer_set, t_curve=2, wheel_radius=3.3, L=28.7):
    """
    Utilizes function that was provided in Cost.py of the Proj 3 Phase 2 files. 
    Get potential moves running action for t_curve seconds
    Variable names have been adjusted slightly, but general mechanics remain the same.
    """
    t    = 0
    cost = 0
    dt   = t_curve / 10
    
    x_new = node[0]
    y_new = node[1]

    theta_new = 3.14 * node[2] / 180 # Convert to Radians
    u_l       = u_l*2*math.pi/60     # Convert to Rad/s
    u_r       = u_r*2*math.pi/60    
    while t < t_curve:
        t = t+dt
        x_new     += (wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new     += (wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        theta_new += (wheel_radius/L) * (u_r - u_l) * dt
        cost       = cost + math.sqrt(math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        
        # Check if move takes us into buffer around wall
        bcux       = int(math.ceil(x_new))
        bcuy       = int(math.ceil(y_new))
        if bcux <0 or bcuy < 0:
            return None
        bcdx = int(x_new)
        bcdy = int(y_new)
        if bcdx < 0 or bcdy < 0:
            return None
        if (bcux,bcuy) in buffer_set or (bcdx,bcdy) in buffer_set:
  
            return None
    
    theta_new = int(180 * theta_new / 3.14) # Convert back to degrees

    
    return (x_new, y_new, theta_new), cost

def ValidMove(node):
    # Check if move is valid
    if((node[0] < 0) or (node[0] >= 540)):
        return False
    elif((node[1] < 0) or (node[1] >= 300)):
        return False
    else:
        return True


def reverse_move(node,movement, t_curve=2, wheel_radius=3.3, L=28.7):
    # Reverse Move for Curve Plotting
    cost  = 0
    dt    = -t_curve/10
    x_new = node[0]
    y_new = node[1]
    u_l = movement[0]
    u_r = movement[1]
    u_l = u_l*2*math.pi/60
    u_r = u_r*2*math.pi/60    
    theta_new = 3.14 * node[2] / 180
    xy_list = [(x_new,y_new)]
    t=0
    #print("reverse Theta Start in rad: ", theta_new)

    
    while t > -t_curve:
        t = t+dt
        theta_new += (wheel_radius/L) * (u_r - u_l) * dt
        x_new += (wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        
        cost = cost + math.sqrt(math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        xy_list.append((round(x_new),round(y_new)))
        
    theta_new = int(180 * theta_new / 3.14)

    return xy_list


def InObjectSpace(x, y):
    """
    Define the object space for all letters/numbers in the maze
    Also used for determining if a set of (x,y) coordinates is present in 
    the action space
    Returns: True if in Object Space, False if not
    """
        
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
    elif( ( (0<=x<=539) and y==0) or ((0<=x<=539) and y==299) ): # x==0 and 0<=y<=299) or (x==539 and (0<=y<=299)) or \
        return True

    # Default case, non-object space    
    else:
        return False


def GeneratePath(CurrentNode, parent, start_state):
    """
    Backtrace the solution path from the goal state to the initial state
    Add all nodes to the "solution" queue
    """
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


def euclidean_distance(node, goal_state):
    """
    Calculate Euclidean Distance between current node and goal state
    Euclidean Distance is the straight line distance between two points
    distance metric used in A* Search
    """
    return math.sqrt((goal_state[0] - node[0])**2 + (goal_state[1] - node[1])**2)


def DrawBoard(rows, cols, pxarray, pallet, C2C, clear, r, screen):
    """
    Draw the initial game board, colors depict:
    White: In object space
    Green: Buffer Zone
    Black: Action Space

    Turn clearance and robot radius used to calculate the total buffer zone that will be placed arround the walls and objects
    Robot will still be depicted on the screen as a "point-robot" as the center-point of the robot is the most import part for calculations

    Inputs:
        rows:    x-axis size
        cols:    y-axis size
        pxarray: pixel array for screen that allows for drawing by point
        pallet:  color dictionary with rgb codes
        C2C:     (obsolete, not used here)
        clear:   clearance needed for turns in mm
        r:       robot raidus in mm

    Outputs:
        buffer_set: set of points that fall within buffer zone, 
                    used later for eliminating potential paths that navigate through the buffer zone 
                    to a point outside of the buffer zone and object space
    """
    buffer_set = set()
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

    return buffer_set



def FillCostMatrix(C2C, pxarray, pallet, thresh, rows, cols, screen):
    for x in range(0, int(rows/thresh)):
        for y in range(0, int(cols/thresh)):
            if((pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["black"])) or \
               (pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["green"]))):
                C2C[x,y] = -1
            else:
                C2C[x,y] = np.inf
                                 
def A_Star(start_node, goal_node, OL, parent, V, C2C, costsum, RPM1, RPM2, 
           t_curve,  pxarray, pallet, screen, goal_threshold, buffer_set,
           wheel_radius=3.3, L= 28.7
           ):
    """
    Run A* Search algorithm over our board
    """

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
        arc_end    = node[1]
        arc_speeds = node[2]
        arc_xy = reverse_move(arc_end, arc_speeds, t_curve, wheel_radius, L)

        pygame.draw.lines(screen,pygame.Color(pallet["blue"]),False,arc_xy,1)
        pygame.display.update()
        
        # Check if popped node is within the goal tolerance region
        # If so, end exploration and backtrace to generate the soluion path
        # If not, apply action set to node and explore the children nodes
        if(euclidean_distance(fixed_node, goal_node) <= goal_threshold ):
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
                test = move_set(fixed_node, action[0], action[1], buffer_set, t_curve, wheel_radius, L)
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



def GetUserInput():
    '''
    Collect input from user:
    start_node: starting coordinates and orientation with total cost (float)
    goal_node:  desired end coordinates as tuple (float)
    step:       step size/movement length in mm (int)
    rradius:    size of robot radius in mm (int)
    start_node, goal_node, step, rradius = GetUserInput()
    '''
    
    unanswered = True
    init_cost = 0.0
    init_speeds = (0,0)
    
    print("Welcome to the obstacle course!")
    print("Here we'll be navigating a Waffle model Turtlebot through a simple arena!")
    print("Before we get this show on the road, we need some information from you!\n")
    print("---------------------------------------------------------------------------\n")
    while unanswered:
        while True:
            start_x     = float(input("Enter the starting x-coordinate [cm] (0-539): "))
            start_y     = 149-float(input("Enter the starting y-coordinate [cm] (-149 to 149): "))
            start_theta = float(input("Enter the starting orientation (0-360 degrees): "))
            if 0 <= start_x <= 539 and 0 <= start_y <= 299 and 0 <= start_theta <= 360:
                break
            print("Error - enter values within range")

        # Check to see if start point falls in obstacle space, reprompt for new
        # coordinates if it does
        if(InObjectSpace(start_x, start_y)):
            print("Sorry, that start point appears to be object space!")
            continue
        
        while True:
            goal_x = float(input("Enter the goal x-coordinate [cm] (0-539): "))
            goal_y = 149-float(input("Enter the goal y-coordinate [cm] (-149 to 149): "))
            if 0 <= goal_x <= 539 and 0 <= start_y <= 299:
                break
            print("Error - enter values within range")
        
        
        # Check to see if goal point falls in obstacle space, reprompt for new
        # coordinates if it does
        if(InObjectSpace(goal_x, goal_y)):
            print("Sorry, that goal point appears to be in object space!")
            continue
        
        while True:
            RPM1 = int(input("Enter the low end wheel speed [rpm] (20-50): "))
            RPM2 = int(input("Enter the high end wheel speed [rpm] (60-100): "))
            if 20 <= RPM1 <= 50 and 60 <= RPM2 <= 100:
                break
            print("Error - enter values within range")

        
        clearance = int(input("Enter the desired clearance space for navigating around buildings (Range: 1-20, must not have decimal point): "))
        if(clearance < 1 or clearance > 20):
            print("Sorry, that clearance is not valid!")
            continue
        
        start_node = [init_cost, (start_x, start_y, start_theta), init_speeds]
        goal = (goal_x,goal_y)
     
        unanswered = False

    return start_node, goal, RPM1, RPM2, clearance

#%%
# Initialize pygame

# Screen dimensions
rows, cols = (540, 300)

# Define Lists
OL = []
CL = {}
index_ctr  = 0
solution   = []
thresh     = 0.5
parent     = {}
costsum    = {}
theta_bins = 72
RPM1 = 50.0
RPM2 = 100.0
r    = 3.3
L    = 28.7
t_curve = 2 # seconds to run curve
goal_threshold = 1.5
robot_radius   = 22

V         = np.zeros((int(rows/thresh), int(cols/thresh), theta_bins))
C2C       = np.zeros((int(rows/thresh), int(cols/thresh)))

# Define colors
pallet = {"white":(255,  255, 255), 
          "black":(0,      0,   0), 
          "green":(4,    217,  13), 
          "blue": (61,   119, 245), 
          "red":  (242,   35,  24)
          }
pygame.init()

# Collect User Input for:
# Start Node
# Goal Node
# Velocities for wheels (RPM1 and RPM2)
# Clearance
start_node, goal_node, RPM1, RPM2, clearance = GetUserInput()

# Define screen size
#clearance   = 18
window_size = (rows+clearance, cols)
screen = pygame.display.set_mode(window_size)
pxarray = pygame.PixelArray(screen)

#start_node = [0.0, (10.0, 150.0, 0), (0,0)]
#goal_node  = (530.0, 149.0)

# Draw board with objects and buffer zone
# rows:      size x-axis (named at one point, and forgot to change)
# cols:      size y-axis
# pallet:    color library with RGB values for drawing on pixel array
# C2C:       obsolete, starting costs set in FillCostMatrix()
# clearance: turn clearance for robot in mm 
# rradius:   robot radius in mm
buffer_set = DrawBoard(rows, cols, pxarray, pallet, C2C, clearance, robot_radius, screen)

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
    
    FillCostMatrix(C2C, pxarray, pallet, thresh, rows, cols, screen)
    
    # Start A_Star algorithm solver, returns game state of either SUCCESS (True) or FAILURE (false)
    alg_state, solution = A_Star(start_node, goal_node, OL, parent, V, C2C, costsum, RPM1, RPM2, t_curve, pxarray,
                                 pallet, screen, goal_threshold, buffer_set)

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
center_y = 149
with open("./part01_waypoints.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([RPM1, RPM2])
    for item in solution:
        x = item[0][0]
        y = center_y - item[0][1]
        writer.writerow([x, y])

# Draw start and goal points; start point will be filled, goal point will be hollow
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(start_node[1][0]), start_node[1][1]), radius=5.0, width=0) # Start node    
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(goal_node[0]), goal_node[1]), radius=5.0, width=1) # Goal node

## Draw solution path
final_path_xyt_list = []
final_path_drawing = []
for item in solution:
    xyt = (int(round(item[0][0])),int(round(item[0][1])),int(round(item[0][2])))
    final_curve = reverse_move(xyt,item[1])
    
    pygame.draw.lines(screen,pygame.Color(pallet["red"]),False,final_curve,2)
    pygame.display.update()

# Freeze screen on completed maze screen until user quits the game
# (press close X on pygame screen)
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            # quit pygame
            pygame.quit()



