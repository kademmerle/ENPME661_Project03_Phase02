# %%
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq


def get_robot_inputs(RPM1, RPM2):
    # Get the robot actions based on the wheel velocities
    # UL: Left wheel velocity
    # UR: Right wheel velocity
    max_omega           = 17.37972   # RPM
    max_linear_velocity = 26 /  1000 # m/s
    robot_radius        = 22 /  1000 # m
    wheel_radius        = 3.3 / 1000 # m
    L                   = 287 / 1000 # m
    control_inputs = []

    action_set = [  [0,    RPM1],
                    [RPM1,    0],
                    [RPM1, RPM1],
                    [0,    RPM2],
                    [RPM2,    0],
                    [RPM2, RPM2],
                    [RPM1, RPM2],
                    [RPM2, RPM1],
    ]

    for action in action_set:
        UL, UR      = action
        UL          = UL * 0.10472 # Convert to rad/s
        UR          = UR * 0.10472 # Convert to rad/s

        control_inputs.append([UL, UR])


    return control_inputs

def get_next_moves(current_pose, dt=0.1, control_inputs=None):
    # Get the next moves based on the robot's current position and action control inputs
    x, y, theta = current_pose
    robot_radius        = 22 /  1000 # m
    wheel_radius        = 3.3 / 1000 # m
    L                   = 287 / 1000 # m
    new_positions = []
    cost = dt# Cost of moving to the next position

    for control_input in control_inputs:
        UL, UR  = control_input
        v        = (wheel_radius / 2) * (UL + UR) # m/s (Linear velocity)
        if UL==UR:
            omega = 0
            dx    = 0.5 * wheel_radius * (UL + UR) * math.cos(theta) * dt # m
            dy    = 0.5 * wheel_radius * (UL + UR) * math.sin(theta) * dt # m
            dtheta = 0
        else:
            omega    = (wheel_radius / L) * (UR - UL) # rad/s (Angular velocity)
            dx        = v/omega * (math.sin(theta + omega*dt) - math.sin(theta)) # m
            dy        = v/omega * (math.cos(theta) - math.cos(theta + omega*dt)) # m
            dtheta    = omega * dt # rad
        x_new         = x + dx*100 # cm
        y_new         = y + dy*100 # cm
        theta_new     = theta + dtheta # Radians
        new_positions.append([(x_new, y_new, theta_new), cost])
    return new_positions


def plot_next_moves(x, y, theta, new_positions):
    # Plot the next moves
    min_x, min_y, max_x, max_y = x, y, x, y
    for new_position, cost in new_positions:
        x_new, y_new, theta_new = new_position
        dx = x_new - x
        dy = y_new - y
        plt.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='blue')
        min_x, min_y, max_x, max_y = min(min_x, x_new), min(min_y, y_new), max(max_x, x_new), max(max_y, y_new)

    plt.xlim([min_x, max_x])
    plt.ylim([min_y, max_y])
    plt.grid()
    plt.show()


def create_cost_matrix(map_img):
    # Create cost matrix with obstacles as -1 and free space as infinity
    # We use [y, x] indexing to match openCV's (row, col) convention
    h, w        = map_img.shape
    cost_matrix = np.ones((h, w)) * np.inf 

    for py in range(h):
        for px in range(w):
            if map_img[py, px] == 0:
                cost_matrix[py, px] = -1
    # Double resolution while keeping the same cost values
    upscaled_cost_matrix = np.repeat(np.repeat(cost_matrix, 2, axis=0), 2, axis=1)

    return upscaled_cost_matrix

def in_rectangle(x, y, xmin, xmax, ymin, ymax):
    # Returns True if (x, y) is inside rectangle defined by (xmin, ymin), (xmax, ymax) corners
    # Coords wrt bottom-left origin
    return (x >= xmin) and (x <= xmax) and (y >= ymin) and (y <= ymax)

def in_wall(x, y, w, h):
    # Check if (x, y) is inside the wall
    # Coords wrt bottom-left origin\
    clearance = .1 # cm

    return (
            y < clearance             # bottom edge
            or y >= h - clearance -1       # top edge
        )

def add_buffer(map_img, buffer_size=5):
    # Add 2 pixels to our map_data by dilating obstacles with a circular kernel with radius=buffer_size
    map_img_copy = map_img.copy()

    # Create Circular Dilation Kernel, for morphology operations, we need a single center pixel, and a 2x2 circle has no center pixel, so we use a 3x3 circle 
    # The center pixel is 1 pixel, and the 8 surrounding pixels extend 1 pixel, so total radius is 2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (buffer_size*2+1, buffer_size*2+1))

    # In OpenCV, white(255) is treated as the foreground, Black(0) is Obstacle Space, so we need to invert the colors
    map_with_clearance = cv2.dilate(255 - map_img_copy, kernel)

    # Invert back (to original representation: obstacles=0, free=255)
    map_img_copy      = 255 - map_with_clearance
    obstacles          = np.where(map_img_copy == 0)
    obstacles          = set(zip(obstacles[1], obstacles[0]))

    # new_obstacles = (map_img == 255) & (map_with_clearance == 0)

    return map_img_copy, obstacles

def draw(map_img, robot_radius):
    h, w = map_img.shape
    for py in range(h):
        for px in range(w):
            x_bl = px
            y_bl = py

            # First Wall
            x1, x2, y1, y2 = 100, 110, 0, 200
            if in_rectangle(x_bl, y_bl, x1, x2, y1, y2):
                map_img[py][px] = 0
            
            x1, x2, y1, y2 = 210, 220, 100, 300
            if in_rectangle(x_bl, y_bl, x1, x2, y1, y2):
                map_img[py][px] = 0

            # Third Wall, part 1
            x1, x2, y1, y2 = 320, 330, 0, 100
            if in_rectangle(x_bl, y_bl, x1, x2, y1, y2):
                map_img[py][px] = 0

            # Third Wall, part 2
            x1, x2, y1, y2 = 320, 330, 200, 300
            if in_rectangle(x_bl, y_bl, x1, x2, y1, y2):
                map_img[py][px] = 0

            # Fourth Wall
            x1, x2, y1, y2 = 430, 440, 0, 200
            if in_rectangle(x_bl, y_bl, x1, x2, y1, y2):
                map_img[py][px] = 0

            # Bottom / Top Wall
            if in_wall(x_bl, y_bl, w, h):
                map_img[py][px] = 0
            
        map_img_with_clearance, obstacles = add_buffer(map_img, robot_radius)

    plt.figure(figsize=(10, 10))
    plt.imshow(map_img_with_clearance, cmap='gray', origin='lower')
    plt.title('Map with Obstacles')
    plt.show()

    return map_img_with_clearance, obstacles
            

def create_map():
    # Create a map of the world
    buffer = 2 # cm add'l buffer beyond the robot radius
    map_width, map_height = 540, 300 # cm
    robot_radius          = 22 + buffer # cm

    map_img = np.ones((map_height, map_width), dtype=np.uint8) * 255 # white

    map_img_with_clearance, obstacles = draw(map_img, robot_radius)
            
            
    return map_img_with_clearance, obstacles


def round_and_get_v_index(node):
    """
    Round x, y coordinates to nearest half to ensure we 
    """
    x           = round(node[0] * 2) / 2
    y           = round(node[1] * 2) / 2
    theta_deg   = theta * 180 / math.pi
    theta_deg_rounded = round(theta_deg / 3) * 3 # round to nearest 3 degrees
    theta_v_idx = theta_deg_rounded % 360 # round to nearest degree
    theta       = round(theta_v_idx * math.pi / 180, 2) # round to nearest degree
 
    x_v_idx     = int(x * 2)
    y_v_idx     = int(y * 2)
   

    return (x, y, theta), x_v_idx, y_v_idx, theta_v_idx


def check_if_visited(V, curr_node_v, stepsize):
    h, w = V.shape[:2]
    y, x, theta = curr_node_v 

    step_size_i = max(int(stepsize // 2), 1)

    x1 = max(x - step_size_i, 0)
    x2 = min(x + step_size_i, w-1)
    y1 = max(y - step_size_i, 0)
    y2 = min(y + step_size_i, h-1)
    # print(x1, x2, y1, y2)

    sum_over_region = np.sum(V[y1:y2, x1:x2, :])
    
    if sum_over_region > 0:
        return True
    else:
        return False


def is_valid_move(node, map):
    h, w = map.shape
    x, y, theta = node

    x, y = int(round(x)), int(round(y))

    if x < 0 or x >= w-1 or y < 0 or y >= h-1:
        return False
    if map[y, x] == 0:
        return False
    return True


def euclidean_distance(node, goal_state):
    """
    Calculate Euclidean Distance between current node and goal state
    Euclidean Distance is the straight line distance between two points
    distance metric used in A* Search
    """
    return math.sqrt((node[0] - goal_state[0])**2 + (node[1] - goal_state[1])**2)


def generate_path(parent, goal_state):
    """
    Generate the path from start state to goal state leveraging parent-child dictionary as input,
    mapping child nodes to parent nodes. We start at goal state and work back to start state, appending 
    each state to a list. We then reverse our list to get correct order of nodes
    from start to goal.

    parent:     Dictionary mapping child states to parent states
    goal_state: Goal state of the puzzle

    Returns:    List of states from the start state to the goal state
    """
    path    = []
    current = goal_state

    while current is not None:
        try: # Try to append current state to path and set current state to parent of current state
            path.append(current)
            current = parent[current]
        except: # If error print the current state and break the loop (This is for debugging, but should not be reached to run DFS or BFS)
            print(current)
            break
    path.reverse()
    return path

def a_star(start_state, goal_state, map_img, cost_matrix, obstacles, r=1, dt=0.1, control_inputs=None):
    """
    Perform A* Search to find shortest path from start state to goal state based on provided map
    and an 8-connected grid.

    Data Structure and Algorithm are same as Dijkstra's Algorithm, but we use Euclideon distance to goal 
    from current state as our heuristic function + cost to come to current state.  


    Parameters:
        start_state:        Initial state of point robot as tuple of (x, y) coordinates
        goal_state:         Goal state of point robot as tuple of (x, y) coordinates
        map_data:           Map with obstacles
        cost_matrix:        Cost matrix with obstacles as -1 and free space as infinity
        obstacles:          Set of obstacle coordinates

    Returns:     
        solution_path:      List of states from the start state to goal state
        cost_to_come:       Dictionary of cost to reach each node
        parent:             Dictionary mapping child states to parent states
        cost_matrix:        Cost matrix with updated costs to reach each node
        explored_path:      List of all nodes expanded by the algorithm in search
        V:                  Visited nodes matrix with 1 for visited nodes and 0 for unvisited nodes
        goal_state_reached: Goal state reached by the algorithm
        
        Steps:
        1. Initialize Open List (priority queue) and Closed List (cost_to_come dictionary)
        2. Add start state to Open List with cost to come + euclideon distance to reach goal
        3. While Open List is not empty:
            1. Pop node with lowest cost_to_come + cost_to_go from Open List
            2. Check if node is within 1.5 mm of goal state, if it is, generate path and break loop
            3. Check if node has higher cost than previously found cost, if it does, skip and continue
            4. Add node to Closed List
            5. Generate possible moves from current node
            6. For each possible move:
                1. Check if move is valid and not an obstacle
                2. Calculate cost to reach next node
                3. Check if next node has not been visited or if new cost is lower than previous cost to reach node
                4. If so, update cost_to_come, parent, and cost_matrix and add node back to Open List
                5. If not, skip and continue
            7. If no solution found, return None

    """
    solution_path = None
    pq            = []                                          # Open List
    cost_to_come  = {}                                          # Closed List
    explored_path = []                                          # List of all nodes expanded in search
    parent        = {start_state: None}                         # Dictionary to map child->parent to backtrack path to goal state
    f_start       = euclidean_distance(start_state, goal_state) # Heuristic function for start state 
    thresh        = 0.5
    V             = np.zeros(                                   # Visited Nodes
                        (int(map_img.shape[0] / thresh),
                         int(map_img.shape[1] / thresh),
                         120)
                    ) 


    start_state, x_v_idx, y_v_idx, theta_v_idx    = round_and_get_v_index(start_state)
    print("Starting A_Star Search for:")
    print("Start State: ", start_state)
    print("Goal State: ", goal_state)

    cost_to_come[(y_v_idx, x_v_idx, theta_v_idx)] = 0.0       # cost_to_come is our Closed List
    cost_matrix[y_v_idx, x_v_idx]    = f_start                # we'll store cost to reach node + heuristic cost to reach goal
    V[y_v_idx, x_v_idx, theta_v_idx] = 1
    goal_reached = goal_state

    heapq.heappush(pq, (f_start, start_state))   # pq is our Open List

    while pq:
        curr_f, curr_node = heapq.heappop(pq) # Pop node with lowest cost from priority queue

        curr_node_round, curr_x_v_idx, curr_y_v_idx, curr_theta_v_idx = round_and_get_v_index(curr_node) # Round to nearest half 
        curr_cost_node = (curr_y_v_idx, curr_x_v_idx, curr_theta_v_idx) # Get cost node for current node
        V[curr_y_v_idx, curr_x_v_idx, curr_theta_v_idx] = 1
        
        if euclidean_distance(curr_node_round, goal_state) <= 1.5:              # If goal state reached, generate path from start to gaol and break the loop
            solution_path = generate_path(parent, curr_node_round)
            print("Found Solution to Goal:")
            print(goal_state)
            print("Cost: ", cost_to_come[curr_cost_node])
            goal_reached = curr_node_round
            break

        if curr_f > cost_to_come[curr_cost_node] + euclidean_distance(curr_node, goal_state):   # If we've found lower cost for this node, 
            continue                                # skip and don't expand this node
        # else:                                     # Only add node to explored path if it is visited and expanded
        #     explored_path.append(curr_node)       # If we've found a lower cost for the node, then we have already explored it

        possible_moves = get_next_moves(curr_node, dt, control_inputs) # Get possible moves from current node

        for next_node, next_cost in possible_moves:   # For each move, check if it is valid and not an obstacle
            next_node_round, next_x_v_idx, next_y_v_idx, next_theta_v_idx = round_and_get_v_index(next_node)
            next_cost_node = (next_y_v_idx, next_x_v_idx, next_theta_v_idx)
            next_v_node    = (next_y_v_idx, next_x_v_idx, next_theta_v_idx)

            valid_move   = is_valid_move(next_node_round, map_img)
            not_obstacle = (next_node_round[0], next_node_round[1]) not in obstacles

            if valid_move and not_obstacle:     # Check if next node is valid and not on top of an obstacle
                
                # We don't use our heuristic function here, we just use the cost to come to the current node + cost to reach next node
                # This is the parameter we want to minimize, but we use the heuristic function to prioritize our queue
                new_cost = cost_to_come[curr_cost_node] + next_cost

                # Check if next has not been visited or if new cost is lower than previous cost to reach node
                # For cases where we've found a lower cost to reach a node, we update the cost_to_come, parent, and cost_matrix
                # and add the node back-in to the priority queue without removing the old node, if the old node is reached again
                # we skip it with the continue statement above

                visited = (V[next_y_v_idx, next_x_v_idx, next_theta_v_idx] == 1)
                
                if (not visited) or (new_cost < cost_to_come.get(next_v_node, float('inf')) ):
                
                    explored_path.append(next_node_round)
                    cost_to_come[next_cost_node] = new_cost
                    
                    parent[next_node_round]            = curr_node_round
                    # Add Heurstic cost to reach goal to cost to come to current node for prioritization
                    f_next                   = new_cost + euclidean_distance(next_node_round, goal_state)
                    heapq.heappush(pq, (f_next, next_node_round))
                    cost_matrix[next_y_v_idx, next_x_v_idx] = new_cost
                    V[next_y_v_idx, next_x_v_idx, next_theta_v_idx] = 1

    if solution_path is None:
        print("No Solution Found")

    print("A_star Expanded States: ", len(explored_path))

def create_cost_matrix(map_img):
    # Create cost matrix with obstacles as -1 and free space as infinity
    # We use [y, x] indexing to match openCV's (row, col) convention
    h, w        = map_img.shape
    cost_matrix = np.ones((h, w)) * np.inf 

    for py in range(h):
        for px in range(w):
            if map_img[py, px] == 0:
                cost_matrix[py, px] = -1
    # Double resolution while keeping the same cost values
    upscaled_cost_matrix = np.repeat(np.repeat(cost_matrix, 2, axis=0), 2, axis=1)

    return upscaled_cost_matrix

def plot_cost_matrix(cost_matrix, start_state, goal_state,  title="Cost Matrix Heatmap"):
    plt.figure(figsize=(8, 6))
    # Plot the cost matrix as a heatmap
    plt.imshow(cost_matrix, cmap='jet', origin='lower')
    plt.plot(start_state[0]*2, start_state[1]*2, 'ro', label='Start State')
    plt.plot(goal_state[0]*2, goal_state[1]*2, 'go', label='Goal State')
    plt.colorbar(label='Cost Value') # Add colorbar to show range of cost values
    plt.title(title)
    plt.xlabel("X (columns)")
    plt.ylabel("Y (rows)")
    plt.legend(
    loc='upper center',
    bbox_to_anchor=(0.5, -0.4),
    ncol=2
    )
    plt.show()


# %% Run some test points (only one time step plotted so won't be curved)
RPM1 = 5  # RPM
RPM2 = 10 # RPM
control_inputs = get_robot_inputs(RPM1, RPM2)
x     = 0 # cm
y     = 0 # cm
theta = 0 # cm
dt    = 10 # seconds
new_positions = get_next_moves((x, y, theta), control_inputs=control_inputs, dt=dt)
plot_next_moves(x, y, theta, new_positions)

for i in range(10):
    for new_position, cost in new_positions:
        new_positions = get_next_moves(new_position, control_inputs=control_inputs, dt=dt)
    



# %% Setup for A* Search
RPM1, RPM2         = 5, 10 # RPM
map_img, obstacles = create_map()
cost_matrix        = create_cost_matrix(map_img)
control_inputs     = get_robot_inputs(RPM1, RPM2)

start = (0,   150, 0) # cm
goal  = (539, 150, 0) # cm
r     = 1 # Block size for A* Search, not currently used, dt is
dt    = 10 # seconds

first_moves = get_next_moves(start, dt=dt, control_inputs=control_inputs)
min_dist, max_dist, max_theta = 0, 0, 0

for move in first_moves:
    curr_move, curr_cost = move

    dx        = curr_move[0] - start[0]
    dy        = curr_move[1] - start[1]
    dtheta    = curr_move[2] - start[2]
    dist      = euclidean_distance(curr_move, start)
    min_dist  = min(min_dist, dist)
    max_dist  = max(max_dist, dist)
    max_theta = max(max_theta, dtheta)
print(f"Inputs result in Max {max_dist} cm movement per {dt} seconds")
print(f"Inputs result in Min {min_dist} cm movement per {dt} seconds")
print(f"Inputs result in Max {round(max_theta*180/math.pi,1)} degrees rotation per {dt} seconds")
print("selection of dt relative to RPM and grid size is important")


# %% Run A* Search
(solution_path, cost_to_come, parent, cost_matrix, explored_path, V, goal_state_reached
) = a_star(start, goal, map_img, cost_matrix, obstacles, r, dt, control_inputs)

# %%
plot_cost_matrix(cost_matrix, start, goal, title="Cost Matrix Heatmap")