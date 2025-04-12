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

line_list = []

for i in range(10,200,10):
    line_sublist = []
    for n in range(0,150):
        line_sublist.append((n,i))
    line_list.append(line_sublist)

for i in range(0,len(line_list)):
    line_sublist = line_list[i]
    for item in line_sublist:
        x,y = item
        if screen.get_at((x,y)) == pygame.Color(pallet["white"]):

            pxarray[x,y] = pygame.Color(pallet["blue"])

pygame.display.update()
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            # quit pygame
            pygame.quit()