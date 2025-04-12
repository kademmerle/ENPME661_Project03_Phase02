import pygame
import pygame.gfxdraw
import time
import copy
import math
import heapq
import numpy as np
from queue import PriorityQueue
from collections import deque

# Initialize pygame
pygame.init()

# Define screen size
rows, cols = (540,300)
window_size = (rows, cols)
screen = pygame.display.set_mode(window_size)
screen2 = pygame.display.set_mode(window_size)
pxarray = pygame.PixelArray(screen)
bxarray = pygame.PixelArray(screen)
buffer_set = set()

# Define Lists
OL = []
CL = {}
index_ctr = 0
solution = []
thresh = 0.5
clearance = 5
parent = {}
costsum = {}
V = np.zeros((int(rows/thresh), int(cols/thresh), 12))
C2C = np.zeros((int(rows/thresh), int(cols/thresh)))
robot_radius = 22
clearance = 0


pallet = {"white":(255, 255, 255), 
          "black":(0, 0, 0), 
          "green":(4,217,13), 
          "blue":(61,119,245), 
          "red":(242,35,24)
          }

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
    elif((x==0 and 0<=y<=299) or (x==539 and (0<=y<=299)) or \
         ((0<=x<=539) and y==0) or ((0<=x<=539) and y==299)):
        return True

    # Default case, non-object space    
    else:
        return False
    
#def DrawBoard(rows, cols, pxarray, pallet, C2C, clear, r):
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
    print("reverse Theta Start in rad: ", theta_new)
    xy_list = [(x_new,y_new)]
    
    while t > -1:
        t = t+dt
        x_new += (r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        theta_new += (r/L) * (u_r - u_l) * dt
        cost = cost + math.sqrt(math.pow(((r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        xy_list.append((round(x_new),round(y_new)))
        
    theta_new = int(180 * theta_new / 3.14)
   

    #print(x_list[0])
    
    return xy_list

def forward_node(node,movement):
    t = 0
    r = 3.3
    L = 28.7
    cost = 0
    dt = 0.1
    x_new = node[0]
    y_new = node[1]
    u_l = movement[0]
    u_r = movement[1]
    theta_new = 3.14 * node[2] / 180
    print("reverse Theta Start in rad: ", theta_new)
    xy_list = [(x_new,y_new)]
    
    while t < 1:
        t = t+dt
        x_new += (r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        theta_new += (r/L) * (u_r - u_l) * dt
        cost = cost + math.sqrt(math.pow(((r * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((r * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        xy_list.append((round(x_new),round(y_new)))
        
    theta_new = int(180 * theta_new / 3.14)
    
    return xy_list

buff_mod = clearance+robot_radius
for x in range(1,rows-1):
    for y in range(0,cols):
        in_obj = InObjectSpace(x,y)
        if (in_obj):
            pygame.draw.circle(screen,pygame.Color(pallet["green"]),(x,y),buff_mod,0)

        elif(0<y<=buff_mod or (298-buff_mod)<=y<299):
                pxarray[x,y] = pygame.Color(pallet["green"])

for x in range(0,rows):
    for y in range(0,cols):
        if screen.get_at((x,y)) == pygame.Color(pallet["black"]):

            pxarray[x,y] = pygame.Color(pallet["white"])

for x in range(0,rows):
    for y in range(0,cols):
        if InObjectSpace(x,y):
            pxarray[x,y] = pygame.Color(pallet["black"])

for x in range(0,rows):
    for y in range(0,cols):
        if screen.get_at((x,y)) != pygame.Color(pallet["white"]):
            buffer_set.add((x,y))

forward_list = forward_node((50,50,0),(5,10))

pygame.draw.lines(screen,pygame.Color(pallet["blue"]),False,forward_list,1)

pygame.display.update()
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            # quit pygame
            pygame.quit()