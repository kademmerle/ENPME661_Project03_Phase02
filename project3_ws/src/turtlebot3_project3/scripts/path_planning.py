# %%
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math



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

def get_next_moves(x, y, theta, dt=0.1, control_inputs=None):
    # Get the next moves based on the robot's current position and action control inputs
    robot_radius        = 22 /  1000 # m
    wheel_radius        = 3.3 / 1000 # m
    L                   = 287 / 1000 # m
    new_positions = []

    for control_input in control_inputs:
        UL, UR  = control_input
        v        = (wheel_radius / 2) * (UL + UR) # m/s (Linear velocity)
        if UL==UR:
            omega = 0
            dx    = 0.5 * wheel_radius * (UL + UR) * math.cos(theta) * dt
            dy    = 0.5 * wheel_radius * (UL + UR) * math.sin(theta) * dt
            dtheta = 0
        else:
            omega    = (wheel_radius / L) * (UR - UL) # rad/s (Angular velocity)
            dx        = v/omega * (math.sin(theta + omega*dt) - math.sin(theta)) # m
            dy        = v/omega * (math.cos(theta) - math.cos(theta + omega*dt)) # m
            dtheta    = omega * dt # rad
        x_new     = x     + dx # Meters
        y_new     = y     + dy # Meters
        theta_new = theta + dtheta # Radians
        new_positions.append([x_new, y_new, theta_new])
    return new_positions

def plot_next_moves(x, y, theta, new_positions):
    # Plot the next moves
    min_x, min_y, max_x, max_y = x, y, x, y
    for new_position in new_positions:
        x_new, y_new, theta_new = new_position
        dx = x_new - x
        dy = y_new - y
        plt.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='blue')
        min_x, min_y, max_x, max_y = min(min_x, x_new), min(min_y, y_new), max(max_x, x_new), max(max_y, y_new)

    plt.xlim([min_x, max_x])
    plt.ylim([min_y, max_y])
    plt.grid()
    plt.show()


RPM1 = 50
RPM2 = 100
control_inputs = get_robot_inputs(RPM1, RPM2)
x = 0
y = 0
theta = 0
dt = 10
new_positions = get_next_moves(x, y, theta, control_inputs=control_inputs, dt=dt)
plot_next_moves(x, y, theta, new_positions)
parent = {(x, y, theta): None}

for i in range(10):
    for new_position in new_positions:
        x, y, theta = new_position
        new_positions = get_next_moves(x, y, theta, control_inputs=control_inputs, dt=dt)
        plot_next_moves(x, y, theta, new_positions)
    



# %%

# %%
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
    map_width, map_height = 540, 300 # cm
    robot_radius          = 22 # cm

    map_img = np.ones((map_height, map_width), dtype=np.uint8) * 255 # white

    map_img_with_clearance, obstacles = draw(map_img, robot_radius)
            
            
    return map_img_with_clearance, obstacles


# %%
map_img, obstacles = create_map()

# %%
